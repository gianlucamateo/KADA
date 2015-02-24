using System;
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

using System.Threading.Tasks;




namespace KADA
{
    public class _3DProcessor
    {
        public const float POINTTOPOINTWEIGHT = 0.5f;
        public const float POINTTOPLANEWEIGHT = 0.5f;
        public const float MAXROTATION = (float)Math.PI / 3;
        public const float TRANSLATIONWEIGHT = 1f;
        public const float MAX_INLIERDISTANCE = 8;
        private const int ICPITERATIONS = 1;
        private const int WORKERCOUNT = 5;



        public Vector3 oldCenter = Vector3.Zero;

        private readonly bool REQUIRE_HIGH_QUALITY_RESULT = false;
        public bool normalAligner = false;

        private KDTreeWrapper brickWrapper;
        private static XNAMatrix prevR;
        private static bool prevRKnown = false;


        private Vector3 ICPTranslation = Vector3.Zero;


        private Model model;


        private int normalCounter = 0;

        private const double MINICPRATIO = 2.5;

        private PipelineManager manager;
        private PipelineDataContainer dataContainer;


        public _3DProcessor(PipelineManager manager, PipelineDataContainer dataContainer)
        {
            this.dataContainer = dataContainer;
            this.manager = manager;
            this.model = dataContainer.model;
            this.brickWrapper = dataContainer.generateKDTree();

            Thread Stage4 = new Thread(new ThreadStart(() => generateCenter()));
            Stage4.Start();
            //Thread Stage4a = new Thread(new ThreadStart(() => generateCenter()));
            //Stage4a.Start();

            KDTreeWrapper tree = dataContainer.model.generateKDTree();
            ConcurrentQueue<ICPWorker> workers1, workers2;
            workers1 = new ConcurrentQueue<ICPWorker>();
            workers2 = new ConcurrentQueue<ICPWorker>();

            for (int i = 0; i < WORKERCOUNT; i++)
            {
                ICPWorker w = new ICPWorker(i, this.dataContainer);
                w.brickWrapper = tree;
                workers1.Enqueue(w);
            }

            Thread Stage61 = new Thread(new ThreadStart(() => PtPlaneICP(workers1)));
            Stage61.Start();



            Thread Stage5 = new Thread(new ThreadStart(() => scanNormals()));
            Stage5.Start();
            Thread Stage52 = new Thread(new ThreadStart(() => scanNormals()));
            Stage52.Start();
            Thread Stage53 = new Thread(new ThreadStart(() => scanNormals()));
            Stage53.Start();

            List<Thread> Stage7 = new List<Thread>();

            for (int i = 0; i < 1; i++)
            {
                Thread x = new Thread(new ThreadStart(() => tryAlign()));
                x.Start();
                Stage7.Add(x);
            }




        }



        //Stage 4
        public void generateCenter()
        {
            SortedList<int, PipelineContainer> outOfOrder = new SortedList<int, PipelineContainer>();
            int stage = 4;
            int lastFrame = 0;
            bool fromOutOfOrder = false;
            while (this.dataContainer.Run)
            {
                PipelineContainer container = null;
                while (container == null && this.dataContainer.Run)
                {

                    container = null;
                    if (outOfOrder.ContainsKey(lastFrame + 1))
                    {
                        container = outOfOrder[lastFrame + 1];
                        outOfOrder.Remove(lastFrame + 1);
                        fromOutOfOrder = true;
                    }
                    else
                    {
                        fromOutOfOrder = false;
                        if (manager.ProcessingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                        {
                            manager.ProcessingQueues[stage].TryDequeue(out container);
                            if (container == null)
                            {
                                Thread.Sleep(this.dataContainer.SLEEPTIME);
                            }
                        }
                        else
                        {
                            Thread.Sleep(this.dataContainer.SLEEPTIME);
                        }
                    }
                }
                if (container == null)
                {
                    break;
                }

                if (container.Number == lastFrame + 1)
                {
                    lastFrame++;
                }
                else if (fromOutOfOrder == false)
                {
                    outOfOrder.Add(container.Number, container);
                    if (outOfOrder.Count > 10)
                    {
                        System.Diagnostics.Debug.WriteLine("GenerateCenter has " + outOfOrder.Count + ", " + lastFrame);
                        lastFrame++;
                        lastFrame = container.Number;
                        foreach (PipelineContainer c in outOfOrder.Values)
                        {
                            manager.Recycle.Enqueue(c);
                        }
                        outOfOrder.Clear();
                    }
                    continue;
                }





                while (container == null && this.dataContainer.Run)
                {

                    container = null;
                    if (manager.ProcessingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.dataContainer.SLEEPTIME);
                        }
                    }
                    else
                    {
                        Thread.Sleep(this.dataContainer.SLEEPTIME);
                    }
                }
                //container.Timings.Add(DateTime.Now);

                if (this.dataContainer.DeNoiseAndICP)
                {
                    DepthColor[,] dc = container.DC;
                    List<Point> qi = container.Qi;

                    float x = 0, y = 0, z = 0;
                    int counter = 0;
                    DepthColor c;
                    Vector3 center;
                    Point point = new Point();

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
                    container.outliers.Clear();
                    for (int xP = 0; xP < dc.GetLength(0); xP++)
                    {
                        for (int yP = 0; yP < dc.GetLength(1); yP++)
                        {

                            c = dc[xP, yP];
                            if (c.Position.Z != 0)
                            {
                                float dist;
                                Vector3.Distance(ref center, ref c.Position, out dist);
                                if (dist < dataContainer.ICPThreshold)
                                {
                                    x += c.Position.X;
                                    y += c.Position.Y;
                                    z += c.Position.Z;
                                    counter++;
                                    double[] arr = { c.Position.X, c.Position.Y, c.Position.Z };

                                    point.position = c.Position;
                                    point.normal = container.Normals[639 - xP, 479 - yP];
                                    point.brickColorInteger = c.BrickColorInteger;
                                    qi.Add(point);
                                }
                                else
                                {
                                    container.outliers.Add(c.Position);
                                }
                            }
                        }
                    }
                    x /= counter;
                    y /= counter;
                    z /= counter;
                    center = new Vector3(x, y, z);

                    oldCenter = center;



                    container.center = center;

                }

                //container.Timings.Add(DateTime.Now);
                manager.ProcessingQueues[++container.Stage].Enqueue(container);
            }

        }

        //Stage 6;
        private int trackingLostCount = 0, resetCount = 0;
        public void PtPlaneICP(ConcurrentQueue<ICPWorker> workers)
        {
            int stage = 6;
            int icpCount = 0;
            float total = 0;
            Matrix H;


            while (this.dataContainer.Run)
            {
                DateTime start = DateTime.Now;
                PipelineContainer container = null;
                while (container == null && this.dataContainer.Run)
                {

                    container = null;
                    if (manager.ProcessingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.dataContainer.SLEEPTIME);
                        }

                    }
                    else
                    {
                        Thread.Sleep(this.dataContainer.SLEEPTIME);
                    }

                }

                if (container == null)
                {
                    break;
                }



                this.dataContainer.ICPThreshold = model.radius;

                //container.Timings.Add(DateTime.Now);
                if (this.dataContainer.DeNoiseAndICP)
                {
                    if (dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL && container.Number % 3 < 2)
                    {
                        //container.Timings.Add(DateTime.Now);
                        manager.ProcessingQueues[++container.Stage].Enqueue(container);
                        container.R = prevR;
                        continue;
                    }
                    DateTime now = DateTime.Now;
                    DepthColor[,] dc = container.DC;
                    List<Point> qi = container.Qi;

                    DateTime elapsed = DateTime.Now;
                    Vector3 center = container.center;

                    container.ICPInliers = 0;

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
                    container.ICPRatio = 0f;
                    int iterations = 0;
                    bool skip = false;
                    XNAMatrix onlyRot = new XNAMatrix();
                    for (int i = 0; i < ICPITERATIONS; i++)
                    {
                        skip = false;
                        iterations = i;

                        container.ICPInliers = 0;
                        container.ICPOutliers = 0;




                        XNAMatrix.Invert(ref R, out RInv);

                        Matrix A = new Matrix(new double[6, 6]);
                        Vector B = new Vector(new double[6]);
                        H = new Matrix(3, 3);
                        onlyRot = new XNAMatrix();
                        onlyRot.M11 = R.M11;
                        onlyRot.M12 = R.M12;
                        onlyRot.M13 = R.M13;
                        onlyRot.M21 = R.M21;
                        onlyRot.M22 = R.M22;
                        onlyRot.M23 = R.M23;
                        onlyRot.M31 = R.M31;
                        onlyRot.M32 = R.M32;
                        onlyRot.M33 = R.M33;




                        foreach (ICPWorker w in workers)
                        {
                            w.ModelRadius = model.radius;
                            w.center = center;
                            w.dataContainer = this.dataContainer;
                            w.onlyRot = onlyRot;
                            w.RInv = RInv;
                            w.container = container;
                            // w.ICPTranslation = ICPTranslation;
                        }

                        int discard = 0;
                        for (int count = 0; count < qi.Count; count++)
                        {

                            if (discard++ % 1 != 0)
                            {
                                continue;
                            }
                            ICPWorker worker;
                            while (!workers.TryDequeue(out worker))
                            {

                            }
                            int upperLimit = count + 100;
                            for (int innerCount = count; innerCount < qi.Count && innerCount < upperLimit; innerCount++)
                            {
                                worker.input.Enqueue(qi[innerCount]);
                                count++;
                            }
                            workers.Enqueue(worker);
                            #region outdated
                            /*
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
                                while (Vector3.Dot(transformedNormal, Vector3.UnitZ) < this.dataContainer.NORMAL_CULLING_LIMIT)//-0.1f)
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
                                return;
                                //continue;
                            }

                            Vector3 pos = p.position;
                            pos /= 20;
                            vC /= 20;
                            Vector3 n = p.normal; //+new Vector3(0.3f, 0.001f, 0.001f);
                            Vector3 c = Vector3.Cross(pos, n);

                            if (neighbour.CurrentDistance > THRESHOLD)
                            {
                                ICPOutliers++;
                                return;
                                //continue;
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
                            /* if (neighbour.CurrentDistance < MAX_INLIERDISTANCE)
                            {
                                currentICPInliers++;
                            }
                            else
                            {
                                currentICPOutliers++;
                            }*/

                            #endregion
                        }

                        while (workers.ElementAt(workers.Count - 1).input.Count > 0)
                        {
                            Thread.Sleep(1);
                        }

                        float totalWeight = 0;
                        container.outlierCenter = Vector3.Zero;
                        List<Vector3> outliers = new List<Vector3>(5000);
                        int workerOutlierCount = 0;
                        Vector3 workerOutlierSum = Vector3.Zero;
                        foreach (ICPWorker w in workers)
                        {

                            //outliers.AddRange(w.Outliers);
                            Thread.Sleep(1);
                            A.AddInplace(w.A);
                            B.AddInplace(w.B);
                            H.AddInplace(w.H);
                            totalWeight += w.totalWeight;
                            container.ICPOutliers += w.ICPOutliers;
                            container.ICPInliers += w.ICPInliers;
                            w.reset();
                            if (w.OutlierSum.Length() > 1)
                                workerOutlierSum += w.OutlierSum;
                            workerOutlierCount += w.OutlierCount;
                        }
                        Vector3 outlierCenter = Vector3.Zero;

                        foreach (Vector3 outlier in container.outliers)
                        {
                            if(!float.IsNaN(outlier.X))
                            outlierCenter += outlier - center;
                        }
                        if (workerOutlierCount > 0 || container.outliers.Count > 0)
                        {
                            container.outlierCenter = outlierCenter + workerOutlierSum;
                            container.outlierCenter /= (container.outliers.Count + workerOutlierCount);
                        }

                        container.outlierCenter = container.outlierCenter + center;

                        float confidence = 0;
                        Vector3 diff = Vector3.Zero;
                        if (dataContainer.outlierCenters.Count > 3)
                        {
                            foreach (Vector3 vec in dataContainer.outlierCenters)
                            {
                                diff += vec - dataContainer.outlierCenter;
                            }
                            confidence = 1f / diff.Length();
                            if (confidence > 5000)
                            {
                                if ((container.outlierCenter - dataContainer.outlierCenter).Length() < 100)
                                {
                                    dataContainer.outlierCenters.Enqueue(container.outlierCenter);
                                }
                                else
                                {
                                    dataContainer.outlierCenters.Dequeue();
                                }

                            }

                        }
                        else if ((container.outliers.Count + workerOutlierCount) > 50)
                        {
                            dataContainer.outlierCenters.Enqueue(container.outlierCenter);
                        }
                        

                        //if ((container.outliers.Count + workerOutlierCount) > 50)
                        //    dataContainer.outlierCenters.Enqueue(container.outlierCenter);

                       



                        if (dataContainer.outlierCenters.Count > 5)
                            dataContainer.outlierCenters.Dequeue();


                        Vector3 averagedOutlierCenter = Vector3.Zero;

                        foreach (Vector3 vec in dataContainer.outlierCenters)
                        {
                            averagedOutlierCenter += vec;
                        }
                        if (dataContainer.outlierCenters.Count()>0)
                            averagedOutlierCenter /= dataContainer.outlierCenters.Count();
                        container.outlierCenter = averagedOutlierCenter;
                        dataContainer.outlierCenter = averagedOutlierCenter;

                        //container.outlierCenter /= workers.Count;
                        //Console.WriteLine(container.outlierCenter);
                        //container.center = currentCenter;
                        if (totalWeight > 0)
                        {
                            //A.MultiplyInplace(1f / totalWeight);
                            //B.MultiplyInplace(1f / totalWeight);
                            H.MultiplyInplace(1f / totalWeight);
                        }

                        for (int row = 0; row < 6; row++)
                        {
                            Vector tot = A.GetRowVector(row);
                            if (tot.Norm() < 1)
                            {
                                double[,] addArr = new double[6, 6];
                                addArr = A.CopyToArray();
                                addArr[row, row] = 0.001f;
                                A = new Matrix(addArr);
                            }
                        }

                        //Point-to-Point component




                        DotNumerics.LinearAlgebra.SingularValueDecomposition s = new SingularValueDecomposition();
                        Matrix S, U, VT;
                        s.ComputeSVD(H, out S, out U, out VT);

                        Matrix V = VT.Transpose();
                        Matrix UT = U.Transpose();
                        Matrix Xm = V.Multiply(UT);

                        XNAMatrix pointR = new XNAMatrix(
                            (float)Xm[0, 0],
                            (float)Xm[0, 1],
                            (float)Xm[0, 2],
                            0,
                            (float)Xm[1, 0],
                            (float)Xm[1, 1],
                            (float)Xm[1, 2],
                            0,
                            (float)Xm[2, 0],
                            (float)Xm[2, 1],
                            (float)Xm[2, 2],
                            0,
                            0, 0, 0, 1
                            );

                        //pointR = XNAMatrix.Multiply(pointR, pointR);

                        pointR = XNAMatrix.Transpose(pointR);

                        LinearEquations LE = new LinearEquations();
                        Vector X = null;
                        try
                        {
                            X = LE.Solve(A, B);
                        }
                        catch (Exception)
                        {
                            Vector3 prominentNormal = container.NormalsList[2];
                            /*XNAMatrix rot = XNAMatrix.CreateFromAxisAngle(prominentNormal, 0.5f);
                            R = XNAMatrix.Multiply(R, rot);
                            prevR = R;*/
                            skip = true;
                            break;
                        }

                        container.ICPRatio = container.ICPOutliers == 0 ? 1000 : (float)container.ICPInliers / container.ICPOutliers;

                        if (!skip)
                        {
                            float currentWeight = 0.2f + Math.Min(0.8f, 1f / container.ICPRatio);
                            double[] XArrTrans = X.ToArray();

                            X = X.Multiply(POINTTOPLANEWEIGHT * currentWeight);
                            double[] XArr = X.ToArray();

                            XNAMatrix XRot = XNAMatrix.CreateRotationX(Math.Min((float)XArr[0], MAXROTATION));
                            XNAMatrix YRot = XNAMatrix.CreateRotationY(Math.Min((float)XArr[1], MAXROTATION));
                            XNAMatrix ZRot = XNAMatrix.CreateRotationZ(Math.Min((float)XArr[2], MAXROTATION));
                            XNAMatrix RTemp = ZRot * YRot * XRot;

                            Vector3 trans = new Vector3((float)(XArrTrans[3]), (float)(XArrTrans[4]), (float)(XArrTrans[5]));
                            trans *= TRANSLATIONWEIGHT;
                            trans *= model.radius;
                            //this.ICPTranslation += trans;
                            if (trans.Length() > 2)
                            {
                                trans.Normalize();
                            }
                            RTemp.M41 = 0;
                            RTemp.M42 = 0;
                            RTemp.M43 = 0;
                            RTemp.M44 = 1;

                            double pointThetaX = -Math.Atan2(pointR.M32, pointR.M33);
                            double pointThetaY = -Math.Atan2(-pointR.M31, Math.Sqrt(pointR.M32 * pointR.M32 + pointR.M33 * pointR.M33));
                            double pointThetaZ = -Math.Atan2(pointR.M21, pointR.M11);

                            pointThetaX *= Math.Min(POINTTOPOINTWEIGHT * currentWeight, MAXROTATION);
                            pointThetaY *= Math.Min(POINTTOPOINTWEIGHT * currentWeight, MAXROTATION);
                            pointThetaZ *= Math.Min(POINTTOPOINTWEIGHT * currentWeight, MAXROTATION);


                            ZRot = XNAMatrix.CreateRotationZ((float)pointThetaZ);
                            YRot = XNAMatrix.CreateRotationY((float)pointThetaY);

                            XRot = XNAMatrix.CreateRotationX((float)pointThetaX);
                            pointR = ZRot * YRot * XRot;

                            this.ICPTranslation = this.ICPTranslation + trans;

                            RTemp = RTemp * pointR;
                            RTemp = XNAMatrix.CreateTranslation(trans) * RTemp;
                            R = RTemp * R;

                        }
                        if (normalAligner)
                        {
                            this.dataContainer.backgroundEvaluator.NormalInput.Enqueue(container);
                            normalAligner = false;
                        }

                    }

                    if (Math.Abs(R.Determinant() - 1) < 0.001f && !skip)
                    {

                        container.onlyRot = onlyRot;
                        normalCounter++;
                        if (container.ICPRatio > MINICPRATIO || !this.REQUIRE_HIGH_QUALITY_RESULT)
                        {
                            container.R = R;
                        }
                        if (container.ICPRatio > MINICPRATIO)
                        {
                            this.dataContainer.lastConfidentR = R;
                            this.dataContainer.trackingConfidence = TrackingConfidenceLevel.ICPFULL;
                            trackingLostCount = 0;
                        }
                        else if (container.ICPRatio > 0.5f)
                        {
                            if (trackingLostCount > 0)
                            {
                                System.Diagnostics.Debug.WriteLine("regained tracking!");
                            }
                            trackingLostCount = 0;
                            resetCount = 0;
                            this.dataContainer.trackingConfidence = TrackingConfidenceLevel.ICPTENTATIVE;
                        }
                        else
                        {
                            this.dataContainer.trackingConfidence = TrackingConfidenceLevel.NONE;
                            trackingLostCount++;
                        }
                        if (trackingLostCount > 100)
                        {
                            this.reset();
                            trackingLostCount = 0;
                        }
                        prevRKnown = true;
                        prevR = R;

                    }


                    this.dataContainer.recordTick();

                }

                //container.Timings.Add(DateTime.Now);
                manager.ProcessingQueues[++container.Stage].Enqueue(container);
                if (icpCount++ % 90 == 0)
                {
                    float time = total / 90;

                    System.Diagnostics.Debug.WriteLine("ICP TOOK ON AVG: " + time);
                    total = 0;
                }
                else
                {
                    total += (DateTime.Now - start).Milliseconds;
                }
            }


        }

        public void tryAlign()
        {
            int stage = 7;
            bool work = false;
            bool reported = false;

            while (this.dataContainer.Run)
            {
                PipelineContainer container = null;
                while (container == null && this.dataContainer.Run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.ProcessingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.dataContainer.SLEEPTIME);
                        }
                    }
                    else
                    {
                        Thread.Sleep(this.dataContainer.SLEEPTIME);
                    }
                }
                if (container == null)
                {
                    break;
                }
                //container.Timings.Add(DateTime.Now);
                if (this.dataContainer.DeNoiseAndICP || work)
                {
                    XNAMatrix temp;
                    if (this.dataContainer.backgroundEvaluator.NormalOutput.TryDequeue(out temp))
                    {
                        dataContainer.normalRotationAdjustment = temp;
                    }
                    //XNAMatrix onlyRot = container.onlyRot;
                    XNAMatrix rot = dataContainer.prevNormalR;
                    if (dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL || dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPTENTATIVE)
                    {
                        if (dataContainer.lastConfidentR.M11 != 1)
                            rot = dataContainer.lastConfidentR;
                        reported = false;
                    }
                    rot.M14 = 0;
                    rot.M24 = 0;
                    rot.M34 = 0;
                    rot.M41 = 0;
                    rot.M42 = 0;
                    rot.M43 = 0;
                    rot.M44 = 1;
                    container.modelVectors[0] = Vector3.Transform(Vector3.UnitX, rot);
                    container.modelVectors[1] = Vector3.Transform(Vector3.UnitY, rot);
                    container.modelVectors[2] = Vector3.Transform(Vector3.UnitZ, rot);
                    container.modelVectors[0].Normalize();
                    container.modelVectors[1].Normalize();
                    container.modelVectors[2].Normalize();
                    Vector3[] modelVectors = container.modelVectors;
                    dataContainer.modelNormals[0] = container.modelVectors[0];
                    dataContainer.modelNormals[1] = container.modelVectors[1];
                    dataContainer.modelNormals[2] = container.modelVectors[2];
                    if (container.estimatedVectors[0] != Vector3.Zero)
                    {
                        //modelVectors = container.estimatedVectors;
                    }
                    Vector3[] normals = container.NormalsList;
                    if (dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL || dataContainer.trackingConfidence == TrackingConfidenceLevel.ICPTENTATIVE)
                    {
                        /*for (int modelV = 0; modelV < 3; modelV++)
                        {
                            double maxDot = double.MinValue;
                            int currentIndex = 0;
                            for (int xyz = 0; xyz < 3; xyz++)
                            {
                                double currentDot = Math.Abs(Vector3.Dot(modelVectors[modelV], normals[xyz]));
                                if (currentDot > maxDot)
                                {
                                    maxDot = currentDot;
                                    currentIndex = xyz;
                                    container.estimatedVectors[modelV] = normals[xyz];
                                }
                            }
                            dataContainer.normalMappings[modelV] = currentIndex;
                        }*/
                    }


                    for (int modelV = 0; modelV < 3; modelV++)
                    {
                        container.estimatedVectors[modelV] = normals[dataContainer.normalMappings[modelV]];
                    }


                    Vector3 axis;
                    float angle;
                    XNAMatrix normalR1, normalR2, result;

                    axis = Vector3.Cross(modelVectors[1], container.estimatedVectors[1]);

                    float dotProduct = Vector3.Dot(modelVectors[1], container.estimatedVectors[1]);



                    angle = (float)Math.Acos(dotProduct);
                    if (dotProduct < 0)
                    {
                        angle = -((float)Math.PI - angle);
                    }
                    axis.Normalize();
                    normalR1 = XNAMatrix.CreateFromAxisAngle(axis, angle / 10);

                    if (float.IsNaN(axis.X) || float.IsNaN(axis.Y) || float.IsNaN(axis.Z))
                    {
                        normalR1 = XNAMatrix.Identity;
                    }



                    result = normalR1;
                    result = result * dataContainer.prevNormalR;

                    modelVectors[2] = Vector3.Transform(modelVectors[2], normalR1);
                    axis = Vector3.Cross(modelVectors[2], container.estimatedVectors[2]);
                    dotProduct = Vector3.Dot(modelVectors[2], container.estimatedVectors[2]);
                    angle = (float)Math.Acos(dotProduct);

                    if (dotProduct < 0)
                    {
                        angle = -((float)Math.PI - angle);
                    }
                    axis.Normalize();
                    normalR2 = XNAMatrix.CreateFromAxisAngle(axis, angle / 10);
                    if (float.IsNaN(axis.X) || float.IsNaN(axis.Y) || float.IsNaN(axis.Z))
                    {
                        normalR2 = XNAMatrix.Identity;
                    }
                    result = normalR2 * normalR1;
                    result = result * dataContainer.prevNormalR;
                    dataContainer.prevNormalR = result;
                    //container.normalR = dataContainer.prevNormalR;     
                    container.rawNormalR = result;
                    container.normalR = dataContainer.normalRotationAdjustment * container.rawNormalR;
                    if (!reported && dataContainer.trackingConfidence == TrackingConfidenceLevel.NORMALS)
                    {
                        prevR = result;//container.normalR;
                        reported = true;
                    }
                }
                //container.Timings.Add(DateTime.Now);
                manager.ProcessingQueues[++container.Stage].Enqueue(container);
                //manager.enqueue(container);
            }


        }

        public void reset()
        {
            System.Diagnostics.Debug.WriteLine("RESET");
            if (Math.Abs(this.dataContainer.lastConfidentR.Determinant() - 1) < 0.001f)
            {
                prevR = this.dataContainer.lastConfidentR;
            }
            else
            {
                prevRKnown = false;
            }
            //this.ICPTranslation = Vector3.Zero;
            this.oldCenter = Vector3.Zero;
        }


        public Vector3[] kMeans(Vector3[,] normals)
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
            return estimatedNormals;
        }
        //stage 5
        public void scanNormals()
        {
            int stage = 5;
            DepthColor[,] dc;

            Vector3 up, down, left, right;

            Vector3 position;
            Vector3 normal;
            Vector3 verticalAvg, horizontalAvg;
            while (this.dataContainer.Run)
            {
                DateTime start = DateTime.Now;
                PipelineContainer container = null;
                while (container == null && this.dataContainer.Run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.ProcessingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.dataContainer.SLEEPTIME);
                        }

                    }
                    else
                    {
                        Thread.Sleep(this.dataContainer.SLEEPTIME);
                    }

                }
                if (container == null)
                {
                    break;
                }
                //container.Timings.Add(DateTime.Now);
                if (this.dataContainer.DeNoiseAndICP)
                {
                    dc = container.DC;

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


                                if (verticalAvg.Length() > 0 && horizontalAvg.Length() > 0 && normal.Length() > 0)
                                {

                                    normal = -normal;
                                    container.Normals[639 - x, 479 - y] = normal;
                                }
                            }
                        }
                    }
                    container.NormalsList = this.kMeans(container.Normals);
                }
                //container.Timings.Add(DateTime.Now);
                manager.ProcessingQueues[++container.Stage].Enqueue(container);


            }
        }
    }
}


