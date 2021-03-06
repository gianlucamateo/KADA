﻿using System;
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
using Point = KADA.Point;
using KDTree;
using KADA;

namespace KADA
{

    public class ICPWorker
    {
        public ConcurrentQueue<Point> input;
        public Vector3 center;
        public XNAMatrix RInv;
        public XNAMatrix onlyRot;
        public Matrix A;
        public Vector B;
        public Matrix H, HTemp;
        private Thread worker;
        public KDTreeWrapper brickWrapper;
        public PipelineDataContainer dataContainer;
        public PipelineContainer container;
        private int maxCount = 0;
        private int number;
        //private StreamWriter file;
        public float totalWeight;
        private List<BrickColor> possibleColors = new List<BrickColor>();
        public int ICPInliers, ICPOutliers;
        public Vector3 OutlierSum;
        public float ModelRadius = 0;
        public ConcurrentQueue<Point> Outliers;
        public int OutlierCount = 0;
        public double sqDist = 0;
        public ConcurrentDictionary<LocatedBrick, int> matchedPoints;

        Point q, firstGuess;
        Vector3 transformedNormal;

        public ICPWorker(int number, PipelineDataContainer dataContainer)
        {
            this.sqDist = 0;
            this.ICPOutliers = 0;
            this.ICPInliers = 0;
            //this.OutlierAvg = Vector3.Zero;
            this.Outliers = new ConcurrentQueue<Point>();
            this.totalWeight = 0;
            this.dataContainer = dataContainer;
            // this.file = new StreamWriter("ICP_worker" + number + "_" + DateTime.Now.Millisecond + ".txt");
            this.number = number;
            this.reset();
            this.input = new ConcurrentQueue<Point>();
            possibleColors.Add(BrickColor.RED);
            possibleColors.Add(BrickColor.GREEN);
            possibleColors.Add(BrickColor.BLUE);
            possibleColors.Add(BrickColor.YELLOW);
            this.H = new Matrix(3, 3);
            this.HTemp = new Matrix(3, 3);
            this.worker = new Thread(new ThreadStart(() => this.routine()));
            worker.Start();
        }

        public void reset()
        {
            this.OutlierSum = Vector3.Zero;
            OutlierCount = 0;
            /*List<Vector3> outliers = new List<Vector3>(this.Outliers);
            foreach (Vector3 outlier in outliers)
            {
                if (outlier.Length() > ModelRadius)
                {
                    OutlierCount++;
                    this.OutlierSum += outlier;
                }
            }*/
            Point trash;

            while (this.Outliers.Count > 0)
            {
                this.Outliers.TryDequeue(out trash);
            }
            this.sqDist = 0;
            this.ICPOutliers = 0;
            this.ICPInliers = 0;
            this.totalWeight = 0;
            this.input = new ConcurrentQueue<Point>();
            this.A = new Matrix(new double[6, 6]);
            this.B = new Vector(new double[6]);
            this.H = new Matrix(3, 3);
            this.HTemp = new Matrix(3, 3);
        }

        private void routine()
        {

            Point point;
            Vector3 v, p;
            Matrix tmA = new Matrix(6, 6), tmAPoint = new Matrix(6, 6);
            Vector tempB = new Vector(6), tempBPoint = new Vector(6);
            DateTime start = DateTime.Now;
            int count = 0;
            int i = 0;
            double[] vArr = new double[3];
            double maxDistance = dataContainer.ICPThreshold * dataContainer.ICPThreshold;
            NearestNeighbor<Point> neighbour;
            while (this.dataContainer.Run)
            {
                if (this.input.Count > maxCount)
                {
                    maxCount = this.input.Count;
                }
                //point = new Point();
                while (!input.TryDequeue(out point) && this.dataContainer.Run)
                {
                    i++;
                    //start = DateTime.Now;
                    count = 0;

                    if (i > 10)
                    {
                        Thread.Sleep(1);
                    }
                }
                v = point.position;

                //point.inlier = true;

                if (container == null)
                {
                    break;
                }
                i = 0;
                count++;
                p = v - center;
                p = Microsoft.Xna.Framework.Vector3.Transform(p, RInv);
                vArr[0] = p.X;
                vArr[1] = p.Y;
                vArr[2] = p.Z;
                maxDistance = dataContainer.ICPThreshold * dataContainer.ICPThreshold;

                /*if (dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL)
                {
                    maxDistance /=4;
                }*/

                /*if (dataContainer.model.Bricks.Count < 3 && dataContainer.editMode)
                {
                    maxDistance = 4;
                }*/
                while (brickWrapper == null)
                {
                    brickWrapper = dataContainer.model.kdTree;
                }
                neighbour = brickWrapper.NearestNeighbors(vArr, 5, fDistance: maxDistance);
                neighbour.MoveNext();

                transformedNormal = Vector3.Zero;
                firstGuess = neighbour.Current;
                double distance = 0f;

                if (firstGuess.position == Vector3.Zero)
                {
                    this.ICPOutliers++;
                    //this.Outliers.Enqueue(p + center + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));
                    continue;
                }

                //sqDist += neighbour.CurrentDistance;
                q = neighbour.Current;
                bool foundMatch = false;
                int outsiderThreshold = 250;

                if (neighbour.CurrentDistance > outsiderThreshold)
                {
                    this.ICPOutliers++;
                    foundMatch = false;
                    foreach (BrickColor bc in possibleColors)
                    {
                        int number = (int)bc;
                        if ((point.brickColorInteger / number) * number == point.brickColorInteger)
                        {
                            if (q.brickColor == bc)
                            {
                                foundMatch = true;
                            }
                        }
                    }
                    if (foundMatch == true)
                    {
                        sqDist += neighbour.CurrentDistance;
                    }

                    point.position = v;
                    if (point.brickColorInteger == (int)dataContainer.addColor)
                        this.Outliers.Enqueue(point);// + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));
                    continue;
                }
                sqDist += neighbour.CurrentDistance;
                if (dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL)
                {
                    maxDistance = 2 * _3DProcessor.MAX_INLIERDISTANCE;
                }
                if (neighbour.CurrentDistance > maxDistance)
                {
                    this.ICPOutliers++;
                    continue;
                }
                bool found = true;
                //if (dataContainer.trackingConfidence != TrackingConfidenceLevel.NONE)
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
                        distance = neighbour.CurrentDistance;
                    }
                    if (!found)
                    {
                        /*this.ICPOutliers++;

                        this.Outliers.Enqueue(p + center + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));*/
                        continue;
                    }
                }


                q = neighbour.Current;



                if (q.normal == Vector3.Zero || q.position == Vector3.Zero)
                {
                    /*this.ICPOutliers++;

                    this.Outliers.Enqueue(p + center + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));*/

                    continue;
                }
                transformedNormal = Vector3.Transform(q.normal, onlyRot);

                p /= dataContainer.model.radius;
                q.position /= dataContainer.model.radius;

                HTemp[0, 0] = p.X * q.position.X;
                HTemp[0, 1] = p.Y * q.position.X;
                HTemp[0, 2] = p.Z * q.position.X;
                HTemp[1, 0] = p.X * q.position.Y;
                HTemp[1, 1] = p.Y * q.position.Y;
                HTemp[1, 2] = p.Z * q.position.Y;
                HTemp[2, 0] = p.X * q.position.Z;
                HTemp[2, 1] = p.Y * q.position.Z;
                HTemp[2, 2] = p.Z * q.position.Z;


                float dotPro = Math.Abs(Vector3.Dot(transformedNormal, point.normal));
                float weight = dotPro>0?dotPro:0.2f;
                //Console.WriteLine(weight);
                //weight = (1 - weight);
                foundMatch = false;
                foreach (BrickColor bc in possibleColors)
                {
                    int number = (int)bc;
                    if ((point.brickColorInteger / number) * number == point.brickColorInteger)
                    {
                        if (q.brickColor == bc)
                        {
                            weight *= 3f;
                            foundMatch = true;
                        }
                    }
                }
                //if (dataContainer.trackingConfidence != TrackingConfidenceLevel.NONE)
                {
                    if (!foundMatch)
                    {
                        weight = 0f;
                        sqDist -= neighbour.CurrentDistance;
                        if (point.brickColorInteger == (int)dataContainer.addColor && neighbour.CurrentDistance>100) 
                        {
                            this.Outliers.Enqueue(point);// + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));
                        }
                    }
                }

                Vector3 pos = q.position;
                //pos /= 20;
                //vC /= 20;
                Vector3 n = q.normal; //+new Vector3(0.3f, 0.001f, 0.001f);
                Vector3 c = Vector3.Cross(pos, n);

                if (neighbour.CurrentDistance > maxDistance)
                {
                    /*this.ICPOutliers++;
                    //this.OutlierAvg += p;
                    this.Outliers.Enqueue(p + center + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));*/
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
                //System.Diagnostics.Debug.WriteLine(weight);
                tmA.MultiplyInplace(weight);

                
                float pMinqTimesN = Vector3.Dot(pos - p, n);

                tempB[0] = pMinqTimesN * c.X;
                tempB[1] = pMinqTimesN * c.Y;
                tempB[2] = pMinqTimesN * c.Z;
                tempB[3] = pMinqTimesN * n.X;
                tempB[4] = pMinqTimesN * n.Y;
                tempB[5] = pMinqTimesN * n.Z;

                /*tempBPoint[0] = p.Z * (p.Y - q.position.Y) - p.Y * (p.Z - q.position.Z);
                tempBPoint[1] = -p.Z * (p.X - q.position.X) + p.X * (p.Z - q.position.Z);
                tempBPoint[2] = p.Y * (p.X - q.position.X) - p.X * (p.Y - q.position.Y);
                tempBPoint[0] = -p.X + q.position.X;
                tempBPoint[0] = -p.Y + q.position.Y;
                tempBPoint[0] = -p.Z + q.position.Z;*/

                tempB.MultiplyInplace(weight);

                HTemp.MultiplyInplace(weight);

                totalWeight += weight;
                H.AddInplace(HTemp);

                A = A + tmA;// +tmAPoint;

                B = B - tempB;// - tempBPoint;
                if (neighbour.CurrentDistance < _3DProcessor.MAX_INLIERDISTANCE)
                {
                    this.ICPInliers++;
                    point.ConsideredICP = true;
                    try
                    {
                        this.matchedPoints[q.Brick]++;
                    }
                    catch (KeyNotFoundException e)
                    {

                    }
                }
                /*else
                {
                    this.Outliers.Enqueue(p + center + new Vector3(dataContainer.R.M41, dataContainer.R.M42, dataContainer.R.M43));
                    this.ICPOutliers++;
                }*/
            }
        }
    }
}
