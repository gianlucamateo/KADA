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
        public const float MAX_INLIERDISTANCE = 8;
        private const int ICPITERATIONS = 1;
        private const int WORKERCOUNT = 10;
        /*private ConcurrentQueue<DepthColor[,]> renderQueue, processingQueue;
        private ConcurrentQueue<Vector3> centers;
        private ConcurrentQueue<XNAMatrix> rotations;*/
        public Vector3 oldCenter = Vector3.Zero;
        public static readonly float THRESHOLD = 200;
        private readonly bool REQUIRE_HIGH_QUALITY_RESULT = false;
        public bool normalAligner = false;

        private static object ICPSemaphore = new object();

        private KDTreeWrapper brickWrapper;
        private static XNAMatrix prevR;
        private static bool prevRKnown = false;

        // public double ICPInliers = 1, ICPOutliers = 0, ICPRatio = 0;
        private Vector3 ICPTranslation = Vector3.Zero;


        private Model model;


        private int normalCounter = 0;

        private const double MINICPRATIO = 2.5;

        private PipelineManager manager;
        private PipelineDataContainer dataContainer;


        public _3DProcessor(PipelineManager manager, PipelineDataContainer dataContainer)//(ConcurrentQueue<DepthColor[,]> processingQueue, ConcurrentQueue<DepthColor[,]> renderQueue,
        //ConcurrentQueue<Vector3> centers, ConcurrentQueue<XNAMatrix> rotations, ConcurrentQueue<DepthColor[,]> depthPool)
        {
            this.dataContainer = dataContainer;
            this.manager = manager;
            this.model = dataContainer.model;
            this.brickWrapper = dataContainer.generateKDTree();


            //this.g = g;


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

            /*for (int i = 0; i < WORKERCOUNT; i++)
            {
                ICPWorker w = new ICPWorker(i, this.dataContainer);
                w.brickWrapper = tree;
                workers2.Enqueue(w);
            }
            
            
            Thread Stage62 = new Thread(new ThreadStart(() => PtPlaneICP(workers2)));
            Stage62.Start();*/

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


            //Thread Stage52 = new Thread(new ThreadStart(() => PtPlaneICP(workers2)));
            //Stage52.Start();
            /*Thread Stage53 = new Thread(new ThreadStart(() => PtPlaneICP()));
            Stage53.Start();
            Thread Stage54 = new Thread(new ThreadStart(() => PtPlaneICP()));
            Stage54.Start();
            Thread Stage55 = new Thread(new ThreadStart(() => PtPlaneICP()));
            Stage55.Start();*/


        }

        public void exit()
        {
            //g.Exit();
        }

        //Stage 4
        public void generateCenter()
        {
            SortedList<int, PipelineContainer> outOfOrder = new SortedList<int, PipelineContainer>();
            int stage = 4;
            int lastFrame = 0;
            bool fromOutOfOrder = false;
            while (this.dataContainer.run)
            {
                PipelineContainer container = null;
                while (container == null && this.dataContainer.run)
                {
                    //container = manager.dequeue(stage);
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
                        if (manager.processingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                        {
                            manager.processingQueues[stage].TryDequeue(out container);
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

                if (container.number == lastFrame + 1)
                {
                    lastFrame++;
                }
                else if (fromOutOfOrder == false)
                {
                    outOfOrder.Add(container.number, container);
                    if (outOfOrder.Count > 10)
                    {
                        System.Diagnostics.Debug.WriteLine("GenerateCenter has " + outOfOrder.Count + ", " + lastFrame);
                        lastFrame++;
                        lastFrame = container.number;
                        foreach (PipelineContainer c in outOfOrder.Values)
                        {
                            manager.recycle.Enqueue(c);
                        }
                        outOfOrder.Clear();
                    }
                    continue;
                }


                //Removed anti-frame-mess-logic
                /*
                while (container == null && this.dataContainer.run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.processingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.processingQueues[stage].TryDequeue(out container);
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
                */
                if (this.dataContainer.deNoiseAndICP)
                {
                    DepthColor[,] dc = container.dc;
                    List<Point> qi = container.qi;

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
                    for (int xP = 0; xP < dc.GetLength(0); xP++)
                    {
                        for (int yP = 0; yP < dc.GetLength(1); yP++)
                        {
                            if (container.Normals[xP, yP].Length() > 0)
                            {
                                bool bla = true;
                                bla = !bla;
                            }
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
                                    //point = new Point(c.Position, container.Normals[xP,480-yP]);//[639 - xP, 479 - yP]); // check/debug this
                                    point.position = c.Position;
                                    point.normal = container.Normals[639 - xP, 479 - yP];
                                    qi.Add(point);
                                }
                            }
                        }
                    }
                    x /= counter;
                    y /= counter;
                    z /= counter;
                    center = new Vector3(x, y, z);
                    if (ICPTranslation.Length() > 100)
                    {
                        this.reset();
                    }
                    //center += ICPTranslation;
                    oldCenter = center;

                    //ICP

                    container.center = center;
                }
                //manager.enqueue(container);
                container.timings.Add(DateTime.Now);
                manager.processingQueues[++container.stage].Enqueue(container);
            }
            /*if (this.ICPInliers == 0)
            {
                trackingLostCount++;
                this.reset();
            }*/



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

            //this.ICP(center, dc, qi);
        }

        //Stage 6;
        private int trackingLostCount = 0, resetCount = 0;
        public void PtPlaneICP(ConcurrentQueue<ICPWorker> workers)
        {
            int stage = 6;
            int icpCount = 0;
            float total = 0;
            Matrix H;


            while (this.dataContainer.run)
            {
                DateTime start = DateTime.Now;
                PipelineContainer container = null;
                while (container == null && this.dataContainer.run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.processingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.processingQueues[stage].TryDequeue(out container);
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
                /*if (manager.processingQueues[stage].Count > 3)
                {
                    container.R = prevR;
                    container.ICPInliers = dataContainer.ICPOutliers;
                    container.ICPOutliers = dataContainer.ICPOutliers;
                    container.ICPRatio = dataContainer.ICPRatio;
                    manager.processingQueues[++container.stage].Enqueue(container);
                    continue;
                }*/
                if (container == null)
                {
                    break;
                }
                if (this.dataContainer.deNoiseAndICP)
                {
                    DateTime now = DateTime.Now;
                    DepthColor[,] dc = container.dc;
                    List<Point> qi = container.qi;
                    //System.Diagnostics.Debug.WriteLine(qi.Count);
                    DateTime elapsed = DateTime.Now;
                    Vector3 center = container.center;

                    container.ICPInliers = 0;
                    //int currentICPInliers = 0, currentICPOutliers = 0;

                    //H = new Matrix(3, 3);
                    //double[,] HArr = new double[3, 3];
                    //Matrix HTemp = new Matrix(3, 3);
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
                        //currentICPInliers = 0;
                        //currentICPOutliers = 0;
                        container.ICPInliers = 0;
                        container.ICPOutliers = 0;


                        XNAMatrix.Invert(ref R, out RInv);

                        Matrix A = new Matrix(new double[6, 6]);
                        Vector B = new Vector(new double[6]);
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
                        //foreach (Vector3 v in qi)

                        foreach (ICPWorker w in workers)
                        {
                            w.center = center;
                            w.dataContainer = this.dataContainer;
                            w.onlyRot = onlyRot;
                            w.RInv = RInv;
                            w.container = container;
                            w.reset();
                        }
                        //Parallel.ForEach(qi, new ParallelOptions() { MaxDegreeOfParallelism = 20}, v =>
                        int discard = 0;
                        for (int count = 0; count < qi.Count; count++)
                        //foreach (Vector3 v in qi)
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
                        }//);                        

                        /* int finished = 0;
                         while (finished < workers.Count)
                         {
                             finished = 0;
                             foreach (ICPWorker w in workers)
                             {
                                 if (w.input.Count == 0)
                                 {
                                     finished++;
                                 }
                             }
                             Thread.Sleep(1);
                         }*/

                        while (workers.ElementAt(workers.Count - 1).input.Count > 0)
                        {
                            Thread.Sleep(1);
                        }
                        // System.Diagnostics.Debug.WriteLine(DateTime.Now - now);
                        foreach (ICPWorker w in workers)
                        {
                            A = A.Add(w.A);

                            B = B.Add(w.B);
                        }

                        //REMOVED FOR PERFORMANCE TESTING
                        /*if (container.ICPRatio > MINICPRATIO && i > 1)
                        {
                            break;
                        }*/

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
                            //System.Diagnostics.Debug.WriteLine("LE solver encountered an exception");
                            Vector3 prominentNormal = container.NormalsList[2];
                            XNAMatrix rot = XNAMatrix.CreateFromAxisAngle(prominentNormal, 0.5f);
                            R = XNAMatrix.Multiply(R, rot);
                            prevR = R;
                            skip = true;
                            //this.reset();
                            break;
                        }

                        container.ICPRatio = container.ICPOutliers == 0 ? 0 : (float)container.ICPInliers / container.ICPOutliers;

                        if (!skip)
                        {
                            float factor = 1f;

                            double[] XArrTrans = X.ToArray();
                            X = X.Multiply(factor);
                            double[] XArr = X.ToArray();
                            XNAMatrix RTemp = XNAMatrix.CreateRotationZ((float)XArr[2]);

                            XNAMatrix Rot = XNAMatrix.CreateRotationY((float)XArr[1]);
                            RTemp = XNAMatrix.Multiply(RTemp, Rot);
                            Rot = XNAMatrix.CreateRotationX((float)XArr[0]);
                            RTemp = XNAMatrix.Multiply(RTemp, Rot);
                            Vector3 trans = new Vector3((float)(XArrTrans[3]), (float)(XArrTrans[4]), (float)(XArrTrans[5]));
                            //trans /= 4;
                            this.ICPTranslation += trans;
                            if (trans.Length() > 2)
                            {
                                trans.Normalize();
                            }
                            RTemp.M41 = 0;
                            RTemp.M42 = 0;
                            RTemp.M43 = 0;
                            RTemp.M44 = 1;
                            RTemp = XNAMatrix.Multiply(RTemp, XNAMatrix.CreateTranslation(trans));
                            R = XNAMatrix.Multiply(R, RTemp);


                        }
                        if (normalAligner)
                        {
                            this.dataContainer.backgroundEvaluator.NormalInput.Enqueue(container);
                            normalAligner = false;
                        }
                        //this.ICPInliers = currentICPInliers;
                        //this.ICPOutliers = currentICPOutliers;

                        /*if (iterations > 3)
                        {
                            System.Diagnostics.Debug.WriteLine("ICP took " + (DateTime.Now - elapsed) + "("+iterations+" iterations)");
                        }*/
                    }
                    //System.Diagnostics.Debug.WriteLine(R.Determinant());
                    if (Math.Abs(R.Determinant() - 1) < 0.001f && !skip)
                    {
                        //tryAlign(container, onlyRot);
                        container.onlyRot = onlyRot;
                        normalCounter++;
                        if (container.ICPRatio > MINICPRATIO || !this.REQUIRE_HIGH_QUALITY_RESULT)
                        {
                            //normalCounter = 0;
                            //this.rotations.Enqueue(R);
                            container.R = R;
                        }
                        if (container.ICPRatio > MINICPRATIO)
                        {
                            this.dataContainer.lastConfidentR = R;
                            this.dataContainer.trackingConfidence = TrackingConfidenceLevel.ICPFULL;
                            //this.dataContainer.modelUpVector = Vector3.Transform(Vector3.UnitY, onlyRot);
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
                        prevRKnown = true;
                        prevR = R;


                        if (this.dataContainer.ICPInliers == 0)
                        {
                            /*trackingLostCount++;
                             this.reset();*/
                        }
                        else if (this.dataContainer.ICPRatio <= 0.5f)
                        {
                            trackingLostCount++;
                            if (trackingLostCount == 50)
                            {
                                System.Diagnostics.Debug.WriteLine("Soft RESET");
                                this.ICPTranslation = Vector3.Zero;
                                dataContainer.trackingConfidence = TrackingConfidenceLevel.NORMALS;
                                this.dataContainer.backgroundEvaluator.NormalInput.Enqueue(container);
                            }

                        }
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
                    //                    }
                    //this.dataContainer.ICPInliers = this.ICPInliers;
                    //this.dataContainer.ICPOutliers = this.ICPOutliers;
                    //this.dataContainer.ICPRatio = this.ICPRatio;
                    this.dataContainer.recordTick();
                    //System.Diagnostics.Debug.WriteLine(iterations);

                    //container.center = center;
                }
                //manager.enqueue(container);
                container.timings.Add(DateTime.Now);
                manager.processingQueues[++container.stage].Enqueue(container);
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

            while (this.dataContainer.run)
            {
                PipelineContainer container = null;
                while (container == null && this.dataContainer.run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.processingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.processingQueues[stage].TryDequeue(out container);
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

                if (this.dataContainer.deNoiseAndICP || work)
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
                container.timings.Add(DateTime.Now);
                manager.processingQueues[++container.stage].Enqueue(container);
                //manager.enqueue(container);
            }


        }

        public void reset()
        {
            System.Diagnostics.Debug.WriteLine("RESET");
            this.ICPTranslation = Vector3.Zero;
            if (Math.Abs(this.dataContainer.lastConfidentR.Determinant() - 1) < 0.001f)
            {
                prevR = this.dataContainer.lastConfidentR;
            }
            else
            {
                prevRKnown = false;
            }
            this.ICPTranslation = Vector3.Zero;
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
            while (this.dataContainer.run)
            {
                DateTime start = DateTime.Now;
                PipelineContainer container = null;
                while (container == null && this.dataContainer.run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (manager.processingQueues[stage].Count > dataContainer.MINFRAMESINCONTAINER)
                    {
                        manager.processingQueues[stage].TryDequeue(out container);
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

                if (this.dataContainer.deNoiseAndICP)
                {
                    dc = container.dc;

                    //Bitmap bitmap = new Bitmap(640, 480);




                    //int[, ,] bins = new int[6, 6, 6];

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
                                    container.Normals[639 - x, 479 - y] = normal;
                                    //bins[3+(int)Math.Floor(binIndicator.X), 3+(int)Math.Floor(binIndicator.Y), 3+(int)Math.Floor(binIndicator.Y)] += 1;
                                    //bitmap.SetPixel(639 - x, 479 - y, System.Drawing.Color.FromArgb(254, 127 + (int)(normal.X * 127), 127 + (int)(normal.Y * 127), 127 + (int)(normal.Z * 127)));
                                }
                            }
                        }
                    }
                    container.NormalsList = this.kMeans(container.Normals);
                }
                manager.processingQueues[++container.stage].Enqueue(container);
                //return bitmap;

            }
        }
    }
}


