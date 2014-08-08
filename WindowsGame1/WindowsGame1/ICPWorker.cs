using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.Concurrent;
using System.IO;
using Microsoft.Xna.Framework;
using System.Threading;
using Matrix = DotNumerics.LinearAlgebra.Matrix;
using XNAMatrix = Microsoft.Xna.Framework.Matrix;
using Vector = DotNumerics.LinearAlgebra.Vector;
using Point = XYZFileLoader.Point;
using KDTree;
using XYZFileLoader;

namespace KADA
{

    public class ICPWorker
    {
        public ConcurrentQueue<Vector3> input;
        public Vector3 center;
        public XNAMatrix RInv;
        public XNAMatrix onlyRot;
        public Matrix A;
        public Vector B;
        private Thread worker;
        public KDTreeWrapper brickWrapper;
        public PipelineDataContainer dataContainer;
        public PipelineContainer container;
        private int maxCount = 0;
        private int number;
        private StreamWriter file;

        public ICPWorker(int number, PipelineDataContainer dataContainer)
        {
            this.dataContainer = dataContainer;
            this.file = new StreamWriter("ICP_worker" + number+"_"+DateTime.Now.Millisecond +".txt");
            this.number = number;
            this.reset();
            this.input = new ConcurrentQueue<Vector3>();
            this.worker = new Thread(new ThreadStart(() => this.routine()));
            worker.Start();
        }

        public void reset()
        {
            this.input = new ConcurrentQueue<Vector3>();
            this.A = new Matrix(new double[6, 6]);
            this.B = new Vector(new double[6]);
        }

        private void routine()
        {
            Vector3 v;
            Matrix tmA = new Matrix(6,6);
            Vector tempB = new Vector(6);
            DateTime start = DateTime.Now;
            int count = 0;
            int i = 0;
            double[] vArr = new double[3];
            while (this.dataContainer.run)
            {
                if (this.input.Count > maxCount)
                {
                    maxCount = this.input.Count;
                }
                v = Vector3.Zero;
                while (!input.TryDequeue(out v) && this.dataContainer.run)
                {
                    i++;
                    if (count > 0)
                    {
                        //file.WriteLine(count + " Samples took:" + (DateTime.Now - start));
                    }
                    start = DateTime.Now;
                    count = 0;
                    if (i > 10)
                    {
                        Thread.Sleep(2);
                    }
                }
                if (container == null)
                {
                    break;
                }
                i = 0;
                count++;
                Vector3 vC = v - center;
                vC = Microsoft.Xna.Framework.Vector3.Transform(vC, RInv);
                vArr[0] = vC.X;
                vArr[1] = vC.Y;
                vArr[2] = vC.Z;
                NearestNeighbour<Point> neighbour = brickWrapper.NearestNeighbors(vArr, 50, fDistance: _3DProcessor.THRESHOLD);
                neighbour.MoveNext();
                Point p;
                Vector3 transformedNormal = Vector3.Zero;
                Point firstGuess = neighbour.Current;

                bool found = true;
                if (container.ICPRatio > 0.3f)
                {
                    transformedNormal = Vector3.Transform(neighbour.Current.normal, onlyRot);
                    while (Vector3.Dot(transformedNormal, Vector3.UnitZ) < this.dataContainer.NORMAL_CULLING_LIMIT)//-0.1f)
                    {
                        if (neighbour.MoveNext() == false)
                        {
                            found = false;
                            break;
                        }
                        transformedNormal = Vector3.Transform(neighbour.Current.normal, onlyRot);
                    }
                    if (!found)
                    {
                        continue;
                    }
                    
                }

                p = neighbour.Current;
                //transformedNormal = Vector3.Transform(p.normal, onlyRot);
                if (p.normal == Vector3.Zero)
                {
                    //p = firstGuess;
                    
                    container.ICPOutliers++;
                    
                    //return;
                    continue;
                }

                Vector3 pos = p.position;
                pos /= 20;
                vC /= 20;
                Vector3 n = p.normal; //+new Vector3(0.3f, 0.001f, 0.001f);
                Vector3 c = Vector3.Cross(pos, n);

                if (neighbour.CurrentDistance > _3DProcessor.THRESHOLD)
                {
                    container.ICPOutliers++;
                    //return;
                    continue;
                }

                //Matrix tmA = new Matrix(6, 6);
                tmA[0, 0] = c.X * c.X; tmA[0, 1] = c.X * c.Y; tmA[0, 2] = c.X * c.Z; tmA[0, 3] = c.X * n.X; tmA[0, 4] = c.X * n.Y; tmA[0, 5] = c.X * n.Z;
                tmA[1, 0] = tmA[0, 1]; tmA[1, 1] = c.Y * c.Y; tmA[1, 2] = c.Y * c.Z; tmA[1, 3] = c.Y * n.X; tmA[1, 4] = c.Y * n.Y; tmA[1, 5] = c.Y * n.Z;
                tmA[2, 0] = tmA[0, 2]; tmA[2, 1] = tmA[1, 2]; tmA[2, 2] = c.Z * c.Z; tmA[2, 3] = c.Z * n.X; tmA[2, 4] = c.Z * n.Y; tmA[2, 5] = c.Z * n.Z;
                tmA[3, 0] = tmA[0, 3]; tmA[3, 1] = tmA[1, 3]; tmA[3, 2] = tmA[2, 3]; tmA[3, 3] = n.X * n.X; tmA[3, 4] = n.X * n.Y; tmA[3, 5] = n.X * n.Z;
                tmA[4, 0] = tmA[0, 4]; tmA[4, 1] = tmA[1, 4]; tmA[4, 2] = tmA[2, 4]; tmA[4, 3] = tmA[3, 4]; tmA[4, 4] = n.Y * n.Y; tmA[4, 5] = n.Y * n.Z;
                tmA[5, 0] = tmA[0, 5]; tmA[5, 1] = tmA[1, 5]; tmA[5, 2] = tmA[2, 5]; tmA[5, 3] = tmA[3, 5]; tmA[5, 4] = tmA[4, 5]; tmA[5, 5] = n.Z * n.Z;

                //double[] tempB = new double[6];

                float pMinqTimesN = Vector3.Dot(pos - vC, n);

                tempB[0] = pMinqTimesN * c.X;
                tempB[1] = pMinqTimesN * c.Y;
                tempB[2] = pMinqTimesN * c.Z;
                tempB[3] = pMinqTimesN * n.X;
                tempB[4] = pMinqTimesN * n.Y;
                tempB[5] = pMinqTimesN * n.Z;
                
                A = A.Add(tmA);
                B = B.Subtract(tempB);
                if (neighbour.CurrentDistance < _3DProcessor.MAX_INLIERDISTANCE)
                {
                    container.ICPInliers++;
                }
                else
                {
                    container.ICPOutliers++;
                }
            }
        }
    }
}
