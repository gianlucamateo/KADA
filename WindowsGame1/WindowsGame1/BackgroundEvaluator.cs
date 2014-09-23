using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.Xna.Framework;
using System.Collections.Concurrent;
using System.IO;
using Point = XYZFileLoader.Point;

namespace KADA
{
    public class BackgroundEvaluator
    {
        private PipelineDataContainer dataContainer;
        public ConcurrentQueue<PipelineContainer> NormalInput;
        public ConcurrentQueue<Matrix> NormalOutput;
        private Matrix[] possibleRotations;

        public BackgroundEvaluator(PipelineDataContainer dataContainer)
        {
            this.NormalOutput = new ConcurrentQueue<Matrix>();
            this.NormalInput = new ConcurrentQueue<PipelineContainer>();
            this.dataContainer = dataContainer;
            possibleRotations = new Matrix[64];

            Matrix XRot = Matrix.CreateRotationX((float)Math.PI/2);
            Matrix YRot = Matrix.CreateRotationY((float)Math.PI/2);
            Matrix ZRot = Matrix.CreateRotationZ((float)Math.PI/2);

            
            int x = 0, y = 0, z = 0;
            int count = 0;
            for (int i = 0; i < 64; i++)
            {
                Matrix rot = Matrix.Identity;
                for (int xCount = 0; xCount < x; xCount++)
                {
                    rot = XRot * rot;
                }

                for (int yCount = 0; yCount < y; yCount++)
                {
                    rot = YRot * rot;
                }

                for (int zCount = 0; zCount < z; zCount++)
                {
                    rot = ZRot * rot;
                }
                possibleRotations[count++] = rot;
                x++;
                if (x == 4)
                {
                    x = 0;
                    y++;
                    if (y == 4)
                    {
                        y = 0;
                        z++;
                        if (z == 4)
                        {
                            break;
                        }
                    }
                }
            }
            Thread normalAnalyzer = new Thread(new ThreadStart(() => AnalyzeNormal()));
            normalAnalyzer.Start();
           
        }

        private void AnalyzeNormal()
        {
            StreamWriter file = new StreamWriter("rotationScores.txt");
            Thread.Sleep(3000);
            List<Vector3> points = new List<Vector3>();
            PipelineContainer container = null;
            XYZFileLoader.KDTreeWrapper kdTree = dataContainer.model.getKDTree();
            double[] searchArr = new double[3];
            List<double> values = new List<double>();
            Matrix bestRot = Matrix.Identity;
            Vector3 v;
            while (this.dataContainer.run)
            {
                NormalInput.TryDequeue(out container);
                if (container == null)
                {
                    Thread.Sleep(dataContainer.SLEEPTIME);
                    continue;
                }

                List<Point> qi = new List<Point>(container.qi);
                points.Clear();
                Matrix R = container.rawNormalR;
                R = Matrix.Invert(R);
                foreach (Point point in qi)
                {
                    v = point.position;
                    Vector3 p = v - container.center;
                    points.Add(Vector3.Transform(p, R));
                }
                double minDist = double.MaxValue;
                

                Random rand = new Random();
                double totalDist = 0;
                int pointsCalculated = 0;
                int count = 0;
                double variance = 0;
                double avg = 0;
                double measure = 0;
                foreach (Matrix pRot in this.possibleRotations)
                {

                    Matrix rot = Matrix.Invert(pRot);
                    totalDist = 0;
                    pointsCalculated = 0;
                    variance = 0;
                    values.Clear();
                    foreach (Vector3 p in points)
                    {
                        Vector3 point = p;
                        
                        point = Vector3.Transform(point, rot);
                        if (rand.NextDouble() < 0.2)
                        {
                            searchArr[0] = point.X;
                            searchArr[1] = point.Y;
                            searchArr[2] = point.Z;
                            KDTree.NearestNeighbour<Point> neighbour = kdTree.NearestNeighbors(searchArr, 1);
                            neighbour.MoveNext();
                            totalDist += neighbour.CurrentDistance;
                            pointsCalculated++;                            
                            values.Add(neighbour.CurrentDistance);                           
                        }
                    }
                    avg = totalDist/pointsCalculated;

                    foreach (double value in values)
                    {
                        variance += (value - avg) * (value - avg);
                    }
                    
                    variance /= pointsCalculated;
                    measure = avg + 2*variance;
                    file.WriteLine(++count + " : " + avg + " and var: " + variance + " and measure: " + measure);
                    if (measure < minDist)
                    {
                        minDist = measure;
                        bestRot = pRot;
                    }
                }
                NormalOutput.Enqueue(bestRot);


            }
        }
    }

}
