﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using System.Threading;
using System.Collections.Concurrent;
using Microsoft.Xna.Framework;
using Image = System.Drawing.Image;
using System.Drawing;
using Color = System.Drawing.Color;
using KDTree;
using XYZFileLoader;
using Point = XYZFileLoader.Point;
using DotNumerics.LinearAlgebra;
using Matrix = DotNumerics.LinearAlgebra.Matrix;
using XNAMatrix = Microsoft.Xna.Framework.Matrix;
using Model = XYZFileLoader.Model;


using System.Windows.Threading;


namespace KADA
{
    public class _3DProcessor
    {
        public const float MAX_INLIERDISTANCE = 8;
        private ConcurrentQueue<DepthColor[,]> renderQueue, processingQueue;
        private ConcurrentQueue<Vector3> centers;
        private ConcurrentQueue<XNAMatrix> rotations;
        public Vector3 oldCenter = Vector3.Zero;
        private readonly float THRESHOLD = 200;
        private readonly bool REQUIRE_HIGH_QUALITY_RESULT = false;

        private static object ICPSemaphore = new object();

        private KDTreeWrapper brickWrapper;
        private static XNAMatrix prevR;
        private static bool prevRKnown = false;
        private List<Vector3> qi;
        public double ICPInliers = 1, ICPOutliers = 0, ICPRatio = 0;
        private Vector3 ICPTranslation = Vector3.Zero;
        public PCViewer g;
        private XNAMatrix lastConfidentR;
        private Model model;
        System.Windows.Threading.DispatcherOperation pcviewer;

        private int normalCounter = 0;

        private const double MINICPRATIO = 2.5;

        public _3DProcessor(ConcurrentQueue<DepthColor[,]> processingQueue, ConcurrentQueue<DepthColor[,]> renderQueue,
            ConcurrentQueue<Vector3> centers, ConcurrentQueue<XNAMatrix> rotations, ConcurrentQueue<DepthColor[,]> depthPool)
        {
            this.model = new Model();
            this.brickWrapper = model.getKDTree();
            this.renderQueue = renderQueue;
            this.processingQueue = processingQueue;
            this.rotations = rotations;
            this.centers = centers;
            this.qi = new List<Vector3>();
            //this.g = g;
            this.lastConfidentR = new XNAMatrix();

            this.g = new PCViewer(renderQueue, depthPool);//, this);

            pcviewer = Dispatcher.CurrentDispatcher.BeginInvoke(new Action(() =>
            {
                g.Run();
            }));
            g.setModel(this.model);
        }

        public void exit()
        {
            g.Exit();
        }

        public void generateCenter(Object dcIn)
        {
            /*if (this.ICPInliers == 0)
            {
                trackingLostCount++;
                this.reset();
            }*/
            List<Vector3> qi = new List<Vector3>(this.qi);
            DepthColor[,] dc;
            if (processingQueue.TryDequeue(out dc) == false)
                return;
           /* List<Vector3> edgePositions = new List<Vector3>();

            List<Vector2> edgeCoords = new List<Vector2>(1000);
           
            edgeCoords.AddRange(_2DProcessor.edgeCoordinates);//new List<Vector3>(_2DProcessor.edgeCoordinates);
            for (int i = 0; i < edgeCoords.Count; i++)
            {
                Vector2 v = edgeCoords[i];
                Vector3 edgePixel = dc[(int)v.X, (int)v.Y].Position;
                Vector3 direction = g.NormalMap[640 - (int)v.X , 480 -(int)v.Y ];//g.Normals[2];//g.mostConfidentNormal;
                if (direction != Vector3.Zero)
                {
                    bool meg = false;
                }
                if (edgePixel.Z != 0)
                {
                    for (int o = 0; o < 10; o++)
                    {
                        edgePixel -= direction;
                        edgePositions.Add(edgePixel);
                        qi.Add(edgePixel);
                    }
                }
            }
            g.setEdges(edgePositions);*/
            float x = 0, y = 0, z = 0;
            int counter = 0;
            DepthColor c;
            Vector3 center;
            
            if (oldCenter == Vector3.Zero)
            {
                for (int xP = 0; xP < dc.GetLength(0); xP++)
                {
                    for (int yP = 0; yP < dc.GetLength(1); yP++)
                    {
                        c = dc[xP, yP];
                        if (c.Position.Z != 0)
                        {
                            x += c.Position.X;
                            y += c.Position.Y;
                            z += c.Position.Z;
                            counter++;
                        }
                    }
                }
                x /= counter;
                y /= counter;
                z /= counter;
                center = new Vector3(x, y, z);
            }
            else
            {
                center = oldCenter;
            }
            x = 0;
            y = 0;
            z = 0;
            counter = 0;
            for (int xP = 0; xP < dc.GetLength(0); xP++)
            {
                for (int yP = 0; yP < dc.GetLength(1); yP++)
                {
                    c = dc[xP, yP];
                    if (c.Position.Z != 0)
                    {
                        float dist;
                        Vector3.Distance(ref center, ref c.Position, out dist);
                        if (dist < THRESHOLD)
                        {
                            x += c.Position.X;
                            y += c.Position.Y;
                            z += c.Position.Z;
                            counter++;
                            double[] arr = { c.Position.X, c.Position.Y, c.Position.Z };
                            qi.Add(c.Position);
                        }
                    }
                }
            }
            x /= counter;
            y /= counter;
            z /= counter;
            center = new Vector3(x, y, z);
            /* if (ICPTranslation.Length() > 100)
             {
                 this.reset();
             }*/
            center += ICPTranslation;
            oldCenter = center;

            //ICP
            System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(this.PtPlaneICP), new ICPDataContainer(dc, center, qi));
            this.centers.Enqueue(center);
            this.renderQueue.Enqueue(dc);
            //this.ICP(center, dc, qi);
        }

    /*    public void PtPointICP(Object input)
        {
            ICPDataContainer container = (ICPDataContainer)input;
            Vector3 center = container.center;
            DepthColor[,] dc = container.dc;
            List<Vector3> qi = container.qi;
            this.ICPInliers = 0;
            int currentICPInliers = 0, currentICPOutliers = 0;

            Matrix H = new Matrix(3, 3);
            double[,] HArr = new double[3, 3];
            Matrix HTemp = new Matrix(3, 3);
            XNAMatrix R;
            if (!prevRKnown)
            {
                R = XNAMatrix.CreateRotationX(0);
            }
            else
            {
                R = prevR;
            }
            XNAMatrix RInv = XNAMatrix.CreateRotationX(0);
            this.ICPRatio = 0;
            int iterations = 0;
            for (int i = 0; i < 15; i++)
            {
                iterations = i;
                currentICPInliers = 0;
                currentICPOutliers = 0;
                int count = 0;
                if (R.Determinant() == 1)
                {
                    XNAMatrix.Invert(ref R, out RInv);
                }
                foreach (Vector3 v in qi)
                {
                    count++;
                    Vector3 vC = v - center;
                    vC = Microsoft.Xna.Framework.Vector3.Transform(vC, RInv);

                    double[] vArr = new double[] { vC.X, vC.Y, vC.Z };
                    NearestNeighbour<Point> b = brickWrapper.NearestNeighbors(vArr, 1, fDistance: THRESHOLD);
                    b.MoveNext();
                    Point p = b.Current;
                    //this.ICPMisalignment += b.CurrentDistance;
                    Vector3 pos = p.position;
                    HArr[0, 0] = vC.X * pos.X;
                    HArr[0, 1] = vC.Y * pos.X;
                    HArr[0, 2] = vC.Z * pos.X;
                    HArr[1, 0] = vC.X * pos.Y;
                    HArr[1, 1] = vC.Y * pos.Y;
                    HArr[1, 2] = vC.Z * pos.Y;
                    HArr[2, 0] = vC.X * pos.Z;
                    HArr[2, 1] = vC.Y * pos.Z;
                    HArr[2, 2] = vC.Z * pos.Z;
                    HTemp = new Matrix(HArr);
                    if (b.CurrentDistance < THRESHOLD)
                    {
                        H.AddInplace(HTemp);
                        if (b.CurrentDistance < MAX_INLIERDISTANCE)
                        {
                            currentICPInliers++;
                        }
                        else
                        {
                            currentICPOutliers++;
                        }
                    }
                    else
                    {
                        count--;
                    }
                }

                if (this.ICPRatio > MINICPRATIO && i > 2)
                {
                    break;
                }

                H.Multiply(1.0 / (double)count);
                DotNumerics.LinearAlgebra.SingularValueDecomposition s = new SingularValueDecomposition();
                Matrix S, U, VT;
                s.ComputeSVD(H, out S, out U, out VT);

                Matrix V = VT.Transpose();
                Matrix UT = U.Transpose();
                Matrix X = V.Multiply(UT);
                double[,] RArr = X.CopyToArray();
                XNAMatrix RTemp = new XNAMatrix(
                    (float)RArr[0, 0],
                    (float)RArr[0, 1],
                    (float)RArr[0, 2],
                    0,
                    (float)RArr[1, 0],
                    (float)RArr[1, 1],
                    (float)RArr[1, 2],
                    0,
                    (float)RArr[2, 0],
                    (float)RArr[2, 1],
                    (float)RArr[2, 2],
                    0,
                    0, 0, 0, 1
                    );
                RTemp = XNAMatrix.Transpose(RTemp);
                R = XNAMatrix.Multiply(R, RTemp);

                this.ICPInliers = currentICPInliers;
                this.ICPOutliers = currentICPOutliers;
                this.ICPRatio = this.ICPInliers / this.ICPOutliers;
            }
            if (R.Determinant() == 1)
            {
                normalCounter++;
                //if (this.ICPRatio > MINICPRATIO)
                {
                    //normalCounter = 0;
                    this.rotations.Enqueue(R);
                }
                if (normalCounter > 2)
                {
                    //normalCounter++;
                    normalCounter = 0;
                    this.scanNormals();
                }
                prevRKnown = true;
                prevR = R;
            }

            g.ICPInliers = this.ICPInliers;
            g.ICPOutliers = this.ICPOutliers;
            g.ICPRatio = this.ICPRatio;
            System.Diagnostics.Debug.WriteLine(iterations);
        }

        */


        private int trackingLostCount = 0, resetCount = 0;
        public void PtPlaneICP(Object input)
        {
            DateTime elapsed = DateTime.Now;
            ICPDataContainer container = (ICPDataContainer)input;
            Vector3 center = container.center;

            //DepthColor[,] dc = container.dc;
            List<Vector3> qi = container.qi;
            this.ICPInliers = 0;
            int currentICPInliers = 0, currentICPOutliers = 0;

            Matrix H = new Matrix(3, 3);
            double[,] HArr = new double[3, 3];
            Matrix HTemp = new Matrix(3, 3);
            XNAMatrix R;
            if (!prevRKnown)
            {
                R = XNAMatrix.CreateRotationX(0);
            }
            else
            {
                R = prevR;
            }
            XNAMatrix RInv = XNAMatrix.CreateRotationX(0);
            this.ICPRatio = 0;
            int iterations = 0;
            bool skip = false;
            for (int i = 0; i < 4; i++)
            {
                skip = false;
                iterations = i;
                currentICPInliers = 0;
                currentICPOutliers = 0;


                XNAMatrix.Invert(ref R, out RInv);

                Matrix A = new Matrix(new double[6, 6]);
                Vector B = new Vector(new double[6]);
                XNAMatrix onlyRot = new XNAMatrix();
                onlyRot.M11 = R.M11;
                onlyRot.M12 = R.M12;
                onlyRot.M13 = R.M13;
                onlyRot.M21 = R.M21;
                onlyRot.M22 = R.M22;
                onlyRot.M23 = R.M23;
                onlyRot.M31 = R.M31;
                onlyRot.M32 = R.M32;
                onlyRot.M33 = R.M33;
                foreach (Vector3 v in qi)
                {

                    Vector3 vC = v - center;
                    vC = Microsoft.Xna.Framework.Vector3.Transform(vC, RInv);
                    double[] vArr = new double[] { vC.X, vC.Y, vC.Z };
                    NearestNeighbour<Point> neighbour = brickWrapper.NearestNeighbors(vArr, 5, fDistance: THRESHOLD);
                    neighbour.MoveNext();
                    Point p;
                    Vector3 transformedNormal = Vector3.Zero;
                    Point firstGuess = neighbour.Current;
                    if (this.ICPRatio > 0.2f)
                    {
                        transformedNormal = Vector3.Transform(neighbour.Current.normal, onlyRot);
                        while (Vector3.Dot(transformedNormal, Vector3.UnitZ) < g.NORMAL_CULLING_LIMIT)//-0.1f)
                        {
                            if (neighbour.MoveNext() == false)
                            {
                                break;
                            }
                            transformedNormal = Vector3.Transform(neighbour.Current.normal, onlyRot);
                        }
                    }

                    p = neighbour.Current;
                    //transformedNormal = Vector3.Transform(p.normal, onlyRot);
                    if (p.normal == Vector3.Zero)
                    {
                        //p = firstGuess;
                        ICPOutliers++;
                        continue;
                    }

                    Vector3 pos = p.position;
                    pos /= 20;
                    vC /= 20;
                    Vector3 n = p.normal; //+new Vector3(0.3f, 0.001f, 0.001f);
                    Vector3 c = Vector3.Cross(pos, n);

                    if (neighbour.CurrentDistance > THRESHOLD)
                    {
                        ICPOutliers++;
                        continue;
                    }

                    double[,] tmA = new double[6, 6];
                    tmA[0, 0] = c.X * c.X; tmA[0, 1] = c.X * c.Y; tmA[0, 2] = c.X * c.Z; tmA[0, 3] = c.X * n.X; tmA[0, 4] = c.X * n.Y; tmA[0, 5] = c.X * n.Z;
                    tmA[1, 0] = tmA[0, 1]; tmA[1, 1] = c.Y * c.Y; tmA[1, 2] = c.Y * c.Z; tmA[1, 3] = c.Y * n.X; tmA[1, 4] = c.Y * n.Y; tmA[1, 5] = c.Y * n.Z;
                    tmA[2, 0] = tmA[0, 2]; tmA[2, 1] = tmA[1, 2]; tmA[2, 2] = c.Z * c.Z; tmA[2, 3] = c.Z * n.X; tmA[2, 4] = c.Z * n.Y; tmA[2, 5] = c.Z * n.Z;
                    tmA[3, 0] = tmA[0, 3]; tmA[3, 1] = tmA[1, 3]; tmA[3, 2] = tmA[2, 3]; tmA[3, 3] = n.X * n.X; tmA[3, 4] = n.X * n.Y; tmA[3, 5] = n.X * n.Z;
                    tmA[4, 0] = tmA[0, 4]; tmA[4, 1] = tmA[1, 4]; tmA[4, 2] = tmA[2, 4]; tmA[4, 3] = tmA[3, 4]; tmA[4, 4] = n.Y * n.Y; tmA[4, 5] = n.Y * n.Z;
                    tmA[5, 0] = tmA[0, 5]; tmA[5, 1] = tmA[1, 5]; tmA[5, 2] = tmA[2, 5]; tmA[5, 3] = tmA[3, 5]; tmA[5, 4] = tmA[4, 5]; tmA[5, 5] = n.Z * n.Z;

                    double[] tempB = new double[6];

                    float pMinqTimesN = Vector3.Dot(pos - vC, n);

                    tempB[0] = pMinqTimesN * c.X;
                    tempB[1] = pMinqTimesN * c.Y;
                    tempB[2] = pMinqTimesN * c.Z;
                    tempB[3] = pMinqTimesN * n.X;
                    tempB[4] = pMinqTimesN * n.Y;
                    tempB[5] = pMinqTimesN * n.Z;

                    A = A.Add(new Matrix(tmA));



                    B = B.Subtract(new Vector(tempB));
                    if (neighbour.CurrentDistance < MAX_INLIERDISTANCE)
                    {
                        currentICPInliers++;
                    }
                    else
                    {
                        currentICPOutliers++;
                    }
                }

                if (this.ICPRatio > MINICPRATIO && i > 0)
                {
                    break;
                }

                for (int row = 0; row < 6; row++)
                {
                    Vector tot = A.GetRowVector(row);
                    if (tot.Norm() < 1)
                    {
                        double[,] addArr = new double[6, 6];
                        addArr = A.CopyToArray();
                        addArr[row, row] = 1;
                        A = new Matrix(addArr);
                    }
                }

                LinearEquations LE = new LinearEquations();
                Vector X = null;
                try
                {
                    X = LE.Solve(A, B);
                }
                catch (Exception)
                {
                    System.Diagnostics.Debug.WriteLine("LE solver encountered an exception");
                    Vector3 prominentNormal = g.Normals[2];
                    XNAMatrix rot = XNAMatrix.CreateFromAxisAngle(prominentNormal, 0.5f);
                    R = XNAMatrix.Multiply(R, rot);
                    prevR = R;
                    skip = true;
                    //this.reset();
                    break;
                }

                if (!skip)
                {
                    float factor = 0.4f;
                    /*if (g.ICPRatio < MINICPRATIO)
                    {
                        factor = 0.1f;
                    }*/
                    double[] XArrTrans = X.ToArray();
                    X = X.Multiply(factor);
                    double[] XArr = X.ToArray();
                    XNAMatrix RTemp = XNAMatrix.CreateRotationZ((float)XArr[2]);

                    XNAMatrix Rot = XNAMatrix.CreateRotationY((float)XArr[1]);
                    RTemp = XNAMatrix.Multiply(RTemp, Rot);
                    Rot = XNAMatrix.CreateRotationX((float)XArr[0]);
                    RTemp = XNAMatrix.Multiply(RTemp, Rot);
                    Vector3 trans = new Vector3((float)(XArrTrans[3]), (float)(XArrTrans[4]), (float)(XArrTrans[5]));
                    this.ICPTranslation += trans;
                    if (trans.Length() > 2)
                    {
                        trans.Normalize();
                    }
                    RTemp = XNAMatrix.Multiply(RTemp, XNAMatrix.CreateTranslation(trans));
                    R = XNAMatrix.Multiply(R, RTemp);

                }
                this.ICPInliers = currentICPInliers;
                this.ICPOutliers = currentICPOutliers;
                this.ICPRatio = this.ICPInliers / this.ICPOutliers;
                /*if (iterations > 3)
                {
                    System.Diagnostics.Debug.WriteLine("ICP took " + (DateTime.Now - elapsed) + "("+iterations+" iterations)");
                }*/
            }
            //System.Diagnostics.Debug.WriteLine(R.Determinant());
            if (Math.Abs(R.Determinant() - 1) < 0.001f && !skip)
            {
                normalCounter++;
                if (this.ICPRatio > MINICPRATIO || !this.REQUIRE_HIGH_QUALITY_RESULT)
                {
                    //normalCounter = 0;
                    this.rotations.Enqueue(R);
                }
                if (this.ICPRatio > MINICPRATIO)
                {
                    this.lastConfidentR = R;
                }

                if (this.ICPRatio > 0.8f)
                {
                    if (trackingLostCount > 0)
                    {
                        System.Diagnostics.Debug.WriteLine("regained tracking!");
                    }
                    trackingLostCount = 0;
                    resetCount = 0;
                }

                if (normalCounter > 10)
                {
                    //normalCounter++;
                    normalCounter = 0;
                    this.scanNormals();
                }
                prevRKnown = true;
                prevR = R;
            }
            if (this.ICPInliers == 0)
            {
                /*trackingLostCount++;
                this.reset();*/
            }
            else if (this.ICPRatio < 0.8f)
            {
                trackingLostCount++;
                if (trackingLostCount == 15)
                {
                    System.Diagnostics.Debug.WriteLine("Soft RESET");
                    this.ICPTranslation = Vector3.Zero;

                }
                if (trackingLostCount > 50)
                {
                    this.reset();
                }
                if (trackingLostCount > 53)
                {
                    trackingLostCount = 0;
                
                }

                /*if (this.trackingLostCount > 2 && resetCount < 6 && lastConfidentR != XNAMatrix.Identity)
                {

                    int factor = resetCount % 2 == 1 ? 1 : -1;
                    prevR = XNAMatrix.Multiply(this.lastConfidentR, XNAMatrix.CreateFromAxisAngle(g.Normals[2], factor * MathHelper.Pi / 4 * ((float)resetCount / 6f)));
                    //this.rotations.Enqueue(prevR);
                    trackingLostCount = 0;
                    resetCount++;
                    System.Diagnostics.Debug.WriteLine("soft reset");
                }
                if (resetCount > 5)
                {
                    if (resetCount == 5)
                    {
                        System.Diagnostics.Debug.WriteLine("giving up");
                    }
                    resetCount++;
                    //resetCount = 0;
                    //this.reset();
                    //this.lastConfidentR = XNAMatrix.Identity;
                }
                if (resetCount == 8)
                {
                    this.reset();
                }
                //this.reset();
                */
            }
            g.ICPInliers = this.ICPInliers;
            g.ICPOutliers = this.ICPOutliers;
            g.ICPRatio = this.ICPRatio;
            g.recordTick();
            //System.Diagnostics.Debug.WriteLine(iterations);
        }

        public void reset()
        {
            System.Diagnostics.Debug.WriteLine("RESET");
            this.ICPTranslation = Vector3.Zero;
            if (Math.Abs(lastConfidentR.Determinant() - 1) < 0.001f)
            {
                prevR = lastConfidentR;
            }
            else
            {
                prevRKnown = false;
            }
            this.ICPTranslation = Vector3.Zero;
            this.oldCenter = Vector3.Zero;
        }


        public void kMeans(Vector3[,] normals)
        {
            //Bitmap norm = new Bitmap(640, 480);
            Vector3 X = new Vector3(1, 0, 0);
            Vector3 Y = new Vector3(0, 1, 0);
            Vector3 Z = new Vector3(0, 0, 1);

            Vector3[] estimatedNormals = new Vector3[3];
            Color[] colors = { Color.Red, Color.Green, Color.Blue };
            Vector3[] centroids = { X, Y, Z };

            List<Vector3>[] clusters = new List<Vector3>[3];

            for (int i = 0; i < clusters.Length; i++)
            {
                clusters[i] = new List<Vector3>();
            }

            for (int count = 0; count < 5; count++)
            {
                for (int x = 250; x < 640; x++)
                {
                    for (int y = 0; y < 480; y++)
                    {

                        Vector3 currentNormal = normals[x, y];

                        float distance = float.MaxValue;
                        int cluster = int.MaxValue;
                        for (int i = 0; i < clusters.Length; i++)
                        {
                            float temp_distance = float.MaxValue;
                            Vector3.Distance(ref currentNormal, ref centroids[i], out temp_distance);
                            if (temp_distance < distance)
                            {
                                cluster = i;
                                distance = temp_distance;
                            }

                        }
                        if (currentNormal != Vector3.Zero)
                        {
                            clusters[cluster].Add(currentNormal);
                            //norm.SetPixel(x, y, colors[cluster]);
                        }
                    }

                }
                for (int i = 0; i < clusters.Length; i++)
                {
                    Vector3 center = new Vector3(0, 0, 0);
                    foreach (Vector3 v in clusters[i])
                    {
                        center += v;
                    }
                    center.Normalize();
                    centroids[i] = center;

                }
                if (count < 4)
                {
                    for (int i = 0; i < clusters.Length; i++)
                    {
                        clusters[i].Clear();
                    }
                }
            }
            
            estimatedNormals[2] = centroids[2]; // Assuming the Normal pointing directly at the camera to be the most reliable
            int confidentIndex = 0;
            if (clusters[1].Count > clusters[0].Count)
            {
                confidentIndex = 1;
            }
            int maxIndex = 0;
            maxIndex = clusters[1].Count > clusters[2].Count ? 1 : 2;
            maxIndex = clusters[0].Count > Math.Max(clusters[1].Count, clusters[2].Count) ? 0 : maxIndex;
            g.mostConfidentNormal = estimatedNormals[maxIndex];
            int lessConfidentIndex = 1 - confidentIndex;
            estimatedNormals[lessConfidentIndex] = Vector3.Cross(centroids[confidentIndex], estimatedNormals[2]);
            estimatedNormals[confidentIndex] = Vector3.Cross(estimatedNormals[2], estimatedNormals[lessConfidentIndex]);

            //System.Diagnostics.Debug.Write("vector");
            /*for (int i = 0; i < clusters.Length; i++)
            {
                //centroids[i] *= -1;
                //System.Diagnostics.Debug.Write("{" + centroids[i].X + "," + centroids[i].Y + "," + centroids[i].Z + "},");
                //System.Diagnostics.Debug.Write("{" + estimatedNormals[i].X + "," + estimatedNormals[i].Y + "," + estimatedNormals[i].Z + "},");
            }*/
            //norm.Save("normals_clustered.png");
            g.Normals = estimatedNormals;
        }

        public void scanNormals()
        {
            Vector3[,] normals = new Vector3[640, 480];
            Bitmap bitmap = new Bitmap(640, 480);
            DepthColor[,] dc;

            if (!this.processingQueue.TryPeek(out dc))
            {
                return;
            }

            Vector3 up, down, left, right;
            Vector3 zero = new Vector3(0, 0, 0);
            Vector3 position;
            Vector3 normal;
            Vector3 verticalAvg, horizontalAvg;

            int[, ,] bins = new int[6, 6, 6];

            for (int x = 1; x < 640 - 1; x++)
            {
                for (int y = 1; y < 480 - 1; y++)
                {
                    if (dc[x, y].Depth > 0)
                    {
                        position = dc[x, y].Position;
                        up = dc[x, y - 1].Position - position;
                        down = dc[x, y + 1].Position - position;
                        right = dc[x + 1, y].Position - position;
                        left = dc[x - 1, y].Position - position;

                        verticalAvg = up - down;
                        horizontalAvg = left - right;
                        normal = Vector3.Cross(horizontalAvg, verticalAvg);
                        normal.Normalize();
                        //normal.X *= -1;
                        //normal.Y *= -1;
                        //Vector3 binIndicator = normal * 2.99f;

                        if (verticalAvg.Length() > 0 && horizontalAvg.Length() > 0 && normal.Length() > 0)
                        {
                            //normal.X *= -1;
                            //normal.Y *= -1;
                            normal = -normal;
                            normals[639 - x, 479 - y] = normal;
                            //bins[3+(int)Math.Floor(binIndicator.X), 3+(int)Math.Floor(binIndicator.Y), 3+(int)Math.Floor(binIndicator.Y)] += 1;
                            //bitmap.SetPixel(639 - x, 479 - y, System.Drawing.Color.FromArgb(254, 127 + (int)(normal.X * 127), 127 + (int)(normal.Y * 127), 127 + (int)(normal.Z * 127)));
                        }
                    }
                }
            }
            /*bins[3, 3, 3] = 0;
             List<int> greatest = new List<int>();
             for (int x = 0; x < 6; x++)
             {
                 for (int y = 0; y < 6; y++)
                 {
                     for (int z = 0; z < 6; z++)
                     {
                         greatest.Add(bins[x, y, z]);
                     }
                 }
             }
             greatest.Sort();
             greatest.Reverse();
             int first = greatest.ElementAt(1);
             int second = greatest.ElementAt(2);
             int third = greatest.ElementAt(3);*/


            //bitmap.Save("normals.png");
            g.NormalMap = normals;
            this.kMeans(normals);
            //return bitmap;

        }
    }
}


