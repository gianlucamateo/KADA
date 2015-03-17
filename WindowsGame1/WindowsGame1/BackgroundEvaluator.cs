﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Microsoft.Xna.Framework;
using System.Collections.Concurrent;
using System.IO;
using Point = KADA.Point;


namespace KADA
{
    public class BackgroundEvaluator
    {
        private PipelineDataContainer dataContainer;
        public ConcurrentQueue<PipelineContainer> NormalInput;
        public ConcurrentQueue<Matrix> NormalOutput;
        public ConcurrentQueue<List<Point>> PointsInput;
        private Matrix[] possibleRotations;


        public BackgroundEvaluator(PipelineDataContainer dataContainer)
        {
            this.PointsInput = new ConcurrentQueue<List<Point>>();
            this.NormalOutput = new ConcurrentQueue<Matrix>();
            this.NormalInput = new ConcurrentQueue<PipelineContainer>();
            this.dataContainer = dataContainer;
            possibleRotations = new Matrix[64];

            Matrix XRot = Matrix.CreateRotationX((float)Math.PI / 2);
            Matrix YRot = Matrix.CreateRotationY((float)Math.PI / 2);
            Matrix ZRot = Matrix.CreateRotationZ((float)Math.PI / 2);

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
            Thread normalAnalyzer = new Thread(new ThreadStart(() => analyzeNormal()));
            //normalAnalyzer.Start();

            Thread blockTracker = new Thread(new ThreadStart(() => trackAdditionalBlock()));
            blockTracker.Start();

        }

        private void analyzeNormal()
        {
            StreamWriter file = new StreamWriter("rotationScores.txt");
            Thread.Sleep(10000);
            List<Vector3> points = new List<Vector3>();
            PipelineContainer container = null;
            KADA.KDTreeWrapper kdTree = dataContainer.model.getKDTree();
            double[] searchArr = new double[3];
            List<double> values = new List<double>();
            Matrix bestRot = Matrix.Identity;
            Vector3 v;
            while (this.dataContainer.Run)
            {
                NormalInput.TryDequeue(out container);
                if (container == null)
                {
                    Thread.Sleep(dataContainer.SLEEPTIME);
                    continue;
                }

                List<Point> qi = new List<Point>(container.Qi);
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
                    avg = totalDist / pointsCalculated;

                    foreach (double value in values)
                    {
                        variance += (value - avg) * (value - avg);
                    }

                    variance /= pointsCalculated;
                    measure = avg + 2 * variance;
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

        private void trackAdditionalBlock()
        {
            List<Point> PointsList;
            //List<Vector3> Outliers = new List<Vector3>();
            int[,] grid = new int[200, 200], outputGrid = new int[200, 200];
            bool[,] excludeFromErosion = new bool[200, 200];
            float[,] zVals = new float[200, 200];
            while (this.dataContainer.Run)
            {
                PointsInput.TryDequeue(out PointsList);
                if (PointsList == null)
                {
                    Thread.Sleep(3 * dataContainer.SLEEPTIME);
                    continue;
                }
                //Outliers.Clear();
                for (int X = 0; X < grid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < grid.GetLength(1); Y++)
                    {
                        grid[X, Y] = 0;
                        zVals[X, Y] = 0;
                        outputGrid[X, Y] = 0;
                        excludeFromErosion[X, Y] = false;
                    }
                }
                //float minX = float.MaxValue, maxX = 0, minY = float.MaxValue, maxY = 0;
                foreach (Point p in PointsList)
                {

                    int X = (int)Math.Floor(p.position.X / 10) + grid.GetLength(0) / 2;
                    int Y = (int)Math.Floor(p.position.Y / 10) + grid.GetLength(1) / 2;


                    grid[X, Y]++;
                    zVals[X, Y] += p.position.Z;
                    if (p.ConsideredICP)
                    {
                        excludeFromErosion[X, Y] = true;
                    }

                }
                for (int X = 0; X < grid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < grid.GetLength(1); Y++)
                    {
                        if (grid[X, Y] > 0)
                        {
                            zVals[X, Y] /= grid[X, Y];
                            grid[X, Y] = 1;
                        }

                    }
                }



                _2DProcessor.Erode(grid.GetLength(0), grid.GetLength(1), grid, outputGrid);
                for (int X = 0; X < grid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < grid.GetLength(1); Y++)
                    {
                        if (excludeFromErosion[X, Y])
                        {
                            outputGrid[X, Y] = grid[X, Y];
                        }
                    }
                }


                Vector3 pos = Vector3.Zero;
                int count = 0;
                for (int X = 0; X < outputGrid.GetLength(0); X++)
                {
                    for (int Y = 0; Y < outputGrid.GetLength(1); Y++)
                    {
                        if (outputGrid[X, Y] > 0)
                        {
                            count++;
                            Vector3 currentPos = new Vector3((X - outputGrid.GetLength(0) / 2) * 10, (Y - outputGrid.GetLength(0) / 2) * 10, zVals[X, Y]);
                            pos += currentPos;
                        }
                    }
                }
                pos /= count;

                dataContainer.outlierCenter = pos;

                TentativeModel bestGuess = null;
                float distance = float.MaxValue;
                foreach (TentativeModel model in dataContainer.model.tentativeModels)
                {
                    Vector3 p = model.TentativeBrick.center;
                    p = Vector3.Transform(p, dataContainer.R);
                    p += dataContainer.center;
                    float tentativeDistance = Math.Abs((p - pos).Length());
                    if (tentativeDistance < distance)
                    {
                        distance = tentativeDistance;
                        bestGuess = model;
                    }
                }
                dataContainer.tentativeModel = bestGuess;
                Console.WriteLine(distance);

            }


        }


    }

}
