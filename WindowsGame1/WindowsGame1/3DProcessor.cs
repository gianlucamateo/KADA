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
using KADA;
using Point = KADA.Point;
using DotNumerics.LinearAlgebra;
using Matrix = DotNumerics.LinearAlgebra.Matrix;
using XNAMatrix = Microsoft.Xna.Framework.Matrix;
using Model = KADA.Model;

using System.Threading.Tasks;
using System.IO;




namespace KADA
{
    public class _3DProcessor
    {
        public const float POINTTOPOINTWEIGHT = 0.75f;
        public const float POINTTOPLANEWEIGHT = 0.25f;
        public const float MAXROTATION = (float)Math.PI /18;
        public const float TRANSLATIONWEIGHT = 1f;
        public const float MAX_INLIERDISTANCE = 12;
        private const int ICPITERATIONS = 3;
        private const int WORKERCOUNT = 5;
        private const double MINICPMSE = 8;

        private bool beingReset = false; 

        private readonly bool REQUIRE_HIGH_QUALITY_RESULT = false;

        public Vector3 OldCenter = Vector3.Zero, EditModeCenter = Vector3.Zero, CompensateVector = Vector3.Zero;
        public bool NormalAligner = false;


        private static XNAMatrix PrevR;
        private static bool PrevRKnown = false;

        private Vector3 ICPTranslation = Vector3.Zero;

        private Model Model;

        private int NormalCounter = 0;
        private int trackingLostCount = 0, resetCount = 0;
        private DotNumerics.LinearAlgebra.SingularValueDecomposition SVD;
        LinearEquations LE;

        private Thread Stage6;



        private PipelineManager Manager;
        private PipelineDataContainer DataContainer;


        public _3DProcessor(PipelineManager manager, PipelineDataContainer dataContainer)
        {
            this.DataContainer = dataContainer;
            this.Manager = manager;
            this.Model = dataContainer.model;


            Thread Stage4 = new Thread(new ThreadStart(() => GenerateCenter()));
            Stage4.Start();

            //KDTreeWrapper tree;
            ConcurrentQueue<ICPWorker> workers1, workers2;
            workers1 = new ConcurrentQueue<ICPWorker>();
            workers2 = new ConcurrentQueue<ICPWorker>();

            for (int i = 0; i < WORKERCOUNT; i++)
            {
                ICPWorker w = new ICPWorker(i, this.DataContainer);
                w.brickWrapper = dataContainer.model.getKDTree();;
                workers1.Enqueue(w);
            }

            Stage6 = new Thread(new ThreadStart(() => PtPlaneICP(workers1)));
            Stage6.Start();



            Thread Stage5 = new Thread(new ThreadStart(() => scanNormals()));
            Stage5.Start();
            /*Thread Stage51 = new Thread(new ThreadStart(() => scanNormals()));
            Stage51.Start();
            Thread Stage52 = new Thread(new ThreadStart(() => scanNormals()));
            Stage52.Start();
            Thread Stage53 = new Thread(new ThreadStart(() => scanNormals()));
            Stage53.Start();*/
            
            List<Thread> Stage7 = new List<Thread>();
            /*
            for (int i = 0; i < 1; i++)
            {
                Thread x = new Thread(new ThreadStart(() => TryAlign()));
                x.Start();
                Stage7.Add(x);
            }*/
            this.SVD = new SingularValueDecomposition();
            this.LE = new LinearEquations();



        }


        int stage1Counter = 0;
        //Stage 4
        public void GenerateCenter()
        {
            #region Get sorted container

            SortedList<int, PipelineContainer> outOfOrder = new SortedList<int, PipelineContainer>();
            int stage = 4;
            int lastFrame = 0;
            bool fromOutOfOrder = false;
            StreamWriter File;
            DateTime start = DateTime.Now;
            File = new StreamWriter("4 " + stage1Counter++ + ".csv");
            while (this.DataContainer.Run)
            {
                PipelineContainer container = null;
                while (container == null && this.DataContainer.Run)
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
                        if (Manager.ProcessingQueues[stage].Count > DataContainer.MINFRAMESINCONTAINER)
                        {
                            Manager.ProcessingQueues[stage].TryDequeue(out container);
                            if (container == null)
                            {
                                Thread.Sleep(this.DataContainer.SLEEPTIME);
                            }
                        }
                        else
                        {
                            Thread.Sleep(this.DataContainer.SLEEPTIME);
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
                            Manager.Recycle.Enqueue(c);
                        }
                        outOfOrder.Clear();
                    }
                    continue;
                }





                while (container == null && this.DataContainer.Run)
                {
                    
                    container = null;
                    if (Manager.ProcessingQueues[stage].Count > DataContainer.MINFRAMESINCONTAINER)
                    {
                        Manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.DataContainer.SLEEPTIME);
                        }
                    }
                    else
                    {
                        Thread.Sleep(this.DataContainer.SLEEPTIME);
                    }
                }
                //container.Timings.Add(DateTime.Now);
            #endregion
                start = DateTime.Now;
                if (this.DataContainer.DeNoiseAndICP)
                {
                    
                    container.Timings.Add(DateTime.Now);
                    DepthColor[,] dc = container.DC;
                    List<Point> qi = container.Qi;
                    Point point = new Point(Vector3.Zero, Vector3.Zero);


                    Vector3 center;

                    center = CheckOldCenter(dc);

                    DepthColor c;
                    float x = 0, y = 0, z = 0;
                    int counter = 0;
                    container.OutlierPoints.Clear();
                    for (int xP = 0; xP < dc.GetLength(0); xP++)
                    {
                        for (int yP = 0; yP < dc.GetLength(1); yP++)
                        {
                            c = dc[xP, yP];
                            if (c.Position.Z != 0)
                            {
                                float dist;
                                Vector3.Distance(ref center, ref c.Position, out dist);


                                double[] arr = { c.Position.X, c.Position.Y, c.Position.Z };

                                point.position = c.Position;
                                point.normal = container.Normals[639 - xP, 479 - yP];
                                point.brickColorInteger = c.BrickColorInteger;
                                if (dist < DataContainer.ICPThreshold)
                                {
                                    x += c.Position.X;
                                    y += c.Position.Y;
                                    z += c.Position.Z;
                                    counter++;
                                    qi.Add(point);
                                }
                                else
                                {
                                    point.ConsideredICP = false;
                                    container.OutlierPoints.Add(point);
                                }

                            }
                        }
                    }
                    x /= counter;
                    y /= counter;
                    z /= counter;
                    center = new Vector3(x, y, z);

                    OldCenter = center;


                    if (!DataContainer.EditMode)
                    {
                        container.center = center;
                        DataContainer.center = center;
                        EditModeCenter = center;
                    }
                    else
                    {
                        container.center = EditModeCenter;
                        DataContainer.center = EditModeCenter;
                        CompensateVector = EditModeCenter - center;
                    }
                    container.Timings.Add(DateTime.Now);
                    File.WriteLine((DateTime.Now - start).Milliseconds.ToString());
                }

                
                Manager.ProcessingQueues[++container.Stage].Enqueue(container);
            }
            File.Close();
            this.Stage6.Abort();

        }

        private Vector3 CheckOldCenter(DepthColor[,] dc)
        {
            Vector3 center;
            DepthColor c;
            float x = 0, y = 0, z = 0;
            int counter = 0;
            if (OldCenter == Vector3.Zero)
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
                center = OldCenter;
            }
            return center;
        }

        //Stage 6;        
        public void PtPlaneICP(ConcurrentQueue<ICPWorker> workers)
        {
            int stage = 6;
            int icpCount = 0;
            float total = 0;
            Matrix H;
            List<Point> Outliers = new List<Point>();

            StreamWriter File;
            DateTime start1 = DateTime.Now;
            File = new StreamWriter("6 " + stage1Counter++ + ".csv");

            double angleSum = 0;
            List<Point> backgroundData = new List<Point>();

            while (this.DataContainer.Run)
            {

                this.Model = DataContainer.model;

                DateTime start = DateTime.Now;
                PipelineContainer container = null;
                while (container == null && this.DataContainer.Run)
                {

                    container = null;
                    if (Manager.ProcessingQueues[stage].Count > DataContainer.MINFRAMESINCONTAINER)
                    {
                        Manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.DataContainer.SLEEPTIME);
                        }

                    }
                    else
                    {
                        Thread.Sleep(this.DataContainer.SLEEPTIME);
                    }

                }

                if (container == null)
                {
                    break;
                }

                
                this.DataContainer.recordTick();
                this.DataContainer.ICPThreshold = Math.Max(2*Model.radius, 100);
                start = DateTime.Now;
                //container.Timings.Add(DateTime.Now);
                if (this.DataContainer.DeNoiseAndICP)
                {
                    container.Timings.Add(DateTime.Now);
                    if (!DataContainer.EditMode)
                    {
                        if (DataContainer.model.tentativeModels.Count > 0)
                        {
                            this.DataContainer.hintString = "'R' resets the tracking - \n'V' for viewer perspective, 'K' for kinect \n'M' activates edit mode";
                        }
                        else
                        {
                            this.DataContainer.hintString = "Is this the right model?\n'O' for more tries, 'NUMPAD-0' to abort \n'NUMPAD-5' accepts the addition";
                        }
                    }
                    if (DataContainer.EditMode)
                    {
                        if (DataContainer.Attach)
                        {
                            DataContainer.hintString = "Rotate the model slowly - \nWorks best if the model is left on the desk";
                        }
                        else if (DataContainer.removalInitiated)
                        {
                            DataContainer.hintString = "Remove a brick - Do not move the model\nPress 'RIGHT-Shift' when done";
                        }
                        else
                        {
                            String hintString = "";
                            if (DataContainer.model.Bricks.Count < 3)
                            {
                                hintString += "Tracking is locked below 3 bricks (Do not move the model) \n";
                            }
                            hintString += "Place new Brick well visible for kinect\nPress 'Enter' when you are done\n";
                            if (DataContainer.model.Bricks.Count > 2)
                            {
                                hintString += "You can also initiate removal: Press 'RIGHT-Shift'";
                            }
                            this.DataContainer.hintString = hintString;
                        }
                    }
                    //Skip some frames if locked
                    /*if (DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL && !DataContainer.EditMode && container.Number % 3 < 2)
                    {
                        container.Timings.Add(DateTime.Now);
                        Manager.ProcessingQueues[++container.Stage].Enqueue(container);
                        container.R = PrevR;
                        continue;
                    }*/
                    
                    DateTime now = DateTime.Now;
                    DepthColor[,] dc = container.DC;
                    List<Point> qi = container.Qi;

                    DateTime elapsed = DateTime.Now;
                    Vector3 center = container.center;

                    container.ICPInliers = 0;

                    XNAMatrix R;
                    if (!PrevRKnown)
                    {
                        R = XNAMatrix.CreateRotationX(0);
                    }
                    else
                    {
                        R = PrevR;
                    }
                    XNAMatrix RInv = XNAMatrix.CreateRotationX(0);
                    container.ICPMSE = 0f;
                    int iterations = 0;
                    bool skip = false;
                    XNAMatrix onlyRot = new XNAMatrix();
                    ConcurrentDictionary<LocatedBrick, int> localMatchedPoints = new ConcurrentDictionary<LocatedBrick, int>();
                    foreach (LocatedBrick b in Model.Bricks)
                    {
                        localMatchedPoints.GetOrAdd(b,0);
                    }
                    for (int i = 0; i < ICPITERATIONS && DataContainer.Run; i++)
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
                            w.brickWrapper = Model.getKDTree();
                            w.ModelRadius = Model.radius;
                            w.center = center;
                            w.dataContainer = this.DataContainer;
                            w.onlyRot = onlyRot;
                            w.RInv = RInv;
                            w.container = container;
                            w.matchedPoints = localMatchedPoints;
                            // w.ICPTranslation = ICPTranslation;
                        }


                        int reducer = (int)(DataContainer.ICPInliers + DataContainer.ICPOutliers) / 500 + 1;
                        for (int count = 0; count < qi.Count; count++)
                        {

                            ICPWorker worker;
                            while (!workers.TryDequeue(out worker))
                            {

                            }
                            int upperLimit = count + 50;
                            for (int innerCount = count; innerCount < qi.Count && innerCount < upperLimit; innerCount++)
                            {
                                if (innerCount % reducer == 0)
                                {
                                    worker.input.Enqueue(qi[innerCount]);
                                    count++;
                                }
                            }
                            workers.Enqueue(worker);
                        }

                        while (workers.ElementAt(workers.Count - 1).input.Count > 0)
                        {
                            Thread.Sleep(1);
                        }

                        float totalWeight = 0;
                        container.outlierCenter = Vector3.Zero;

                        Outliers.Clear();

                        foreach (ICPWorker w in workers)
                        {

                            A.AddInplace(w.A);
                            B.AddInplace(w.B);
                            H.AddInplace(w.H);
                            totalWeight += w.totalWeight;
                            container.ICPOutliers += w.ICPOutliers;
                            container.ICPInliers += w.ICPInliers;
                            while (w.Outliers.Count > 0)
                            {
                                Point v;
                                w.Outliers.TryDequeue(out v);
                                //Point p = new Point(v, Vector3.Zero);
                                v.ConsideredICP = true;
                                Outliers.Add(v);
                            }
                            container.ICPMSE += (float)w.sqDist;
                            w.reset();

                        }
                        container.ICPMSE /= (container.ICPInliers + container.ICPOutliers);
                        Outliers.AddRange(container.OutlierPoints);
                        Vector3 outlierCenter = Vector3.Zero;
                        DataContainer.matchedPoints = new ConcurrentDictionary<LocatedBrick, int>(localMatchedPoints);

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

                        Matrix S, U, VT;
                        SVD.ComputeSVD(H, out S, out U, out VT);

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

                        pointR = XNAMatrix.Transpose(pointR);



                        Vector X = null;
                        try
                        {
                            X = LE.Solve(A, B);
                        }
                        catch (Exception e)
                        {
                            Console.WriteLine(e);
                            Vector3 prominentNormal = container.NormalsList[2];
                            skip = true;
                            break;
                        }

                        //container.ICPRatio = container.ICPOutliers == 0 ? 1000 : (float)container.ICPInliers / container.ICPOutliers;

                        if (!skip)
                        {
                            float currentWeight = 1;//0.2f + Math.Min(0.8f, 1f / container.ICPRatio);


                            //X = X.Multiply(POINTTOPLANEWEIGHT * currentWeight);
                            X[0] *= POINTTOPLANEWEIGHT * currentWeight;
                            X[1] *= POINTTOPLANEWEIGHT * currentWeight;
                            X[2] *= POINTTOPLANEWEIGHT * currentWeight;

                            float maxRotation = MAXROTATION;
                            if (DataContainer.EditMode)
                            {
                                if (DataContainer.model.tentativeModels.Count == 0 && DataContainer.model.computingTentative == false)
                                {
                                    DataContainer.model.ComputeTentativeBricks();
                                }
                                maxRotation /= 2;
                            }
                            double[] XArr = X.ToArray();
                            Vector3 trans = new Vector3((float)(XArr[3]), (float)(XArr[4]), (float)(XArr[5]));
                            if (Model.Bricks.Count < 2 || (DataContainer.EditMode && Model.Bricks.Count < 3))
                            {
                                XArr[0] = 0;
                                XArr[2] = 0;
                                //XArr[4] = 0;
                            }




                            XNAMatrix XRot = XNAMatrix.CreateRotationX(Math.Min((float)XArr[0], maxRotation));
                            XNAMatrix YRot = XNAMatrix.CreateRotationY(Math.Min((float)XArr[1], maxRotation));
                            XNAMatrix ZRot = XNAMatrix.CreateRotationZ(Math.Min((float)XArr[2], maxRotation));
                            XNAMatrix RTemp = ZRot * YRot * XRot;

                            float Max = (float)Math.Max(Math.Max((XArr[0]), (XArr[1])), (XArr[2]));
                            float Min = (float)Math.Min(Math.Min((XArr[0]), (XArr[1])), (XArr[2]));

                            float add = Max > (-Min) ? Max : Min;

                            angleSum += add;//((XArr[0]) + (XArr[1]) + (XArr[2])) * POINTTOPLANEWEIGHT;

                            
                            trans *= TRANSLATIONWEIGHT;
                            trans *= Model.radius;
                            float maxTrans = 5;
                            if (DataContainer.EditMode && Model.Bricks.Count < 3)
                            {
                                float dist = Vector3.Dot(trans, DataContainer.g);
                                trans -= (dist) * DataContainer.g;
                                maxTrans = Model.Bricks.Count / 10f;
                                if (Model.Bricks.Count == 1)
                                {
                                    trans = Vector3.Zero;
                                }
                            }

                            while (trans.Length() > maxTrans)
                            {
                                trans *= 0.5f;
                            }




                            /*if (!DataContainer.editMode&&afterEditMode>0)
                            {
                                trans = -trans + CompensateVector;
                                CompensateVector *= 0.2f;
                                afterEditMode--;
                            }
                            if (DataContainer.editMode)
                            {
                                afterEditMode = 1;
                            }*/
                            /*if (DataContainer.editMode)
                            {
                                trans *= 2;
                            }*/
                            //this.ICPTranslation += trans;
                            /*if (trans.Length() > 2)
                            {
                                trans.Normalize();
                            }*/
                            RTemp.M41 = 0;
                            RTemp.M42 = 0;
                            RTemp.M43 = 0;
                            RTemp.M44 = 1;
                            
                            //Console.WriteLine(pointR);
                            pointR = XNAMatrix.Transpose(pointR);
                            /*double pointThetaX = Math.Atan2(pointR.M32, pointR.M33);
                            double pointThetaY = Math.Atan2(-pointR.M31, Math.Sqrt(pointR.M32 * pointR.M32 + pointR.M33 * pointR.M33));
                            double pointThetaZ = Math.Atan2(pointR.M21, pointR.M11);*/
                            double pointThetaX = Math.Atan(pointR.M32 / pointR.M33);
                            double pointThetaY = -Math.Asin(pointR.M31);
                            double pointThetaZ = Math.Atan(pointR.M21 / pointR.M11);
                            pointR = XNAMatrix.Transpose(pointR);
                            if (Model.Bricks.Count < 2 || (DataContainer.EditMode && Model.Bricks.Count < 3))
                            {
                                pointThetaX = 0;
                                pointThetaZ = 0;
                            }


                            pointThetaX = Math.Min(pointThetaX * currentWeight*POINTTOPOINTWEIGHT, maxRotation);
                            pointThetaY = Math.Min(pointThetaY * currentWeight * POINTTOPOINTWEIGHT, maxRotation);
                            pointThetaZ = Math.Min(pointThetaZ * currentWeight * POINTTOPOINTWEIGHT, maxRotation);


                            Max = (float)Math.Max(Math.Max((pointThetaX), (pointThetaY)), (pointThetaZ));
                            Min = (float)Math.Min(Math.Min((pointThetaX), (pointThetaY)), (pointThetaZ));

                            add = Max > (-Min) ? Max : Min;



                            angleSum += add;//((pointThetaX) + (pointThetaX) + (pointThetaX)) * POINTTOPOINTWEIGHT;

                            ZRot = XNAMatrix.CreateRotationZ((float)pointThetaZ);
                            YRot = XNAMatrix.CreateRotationY((float)pointThetaY);

                            XRot = XNAMatrix.CreateRotationX((float)pointThetaX);
                            pointR = ZRot * YRot * XRot;
                            
                            this.ICPTranslation = this.ICPTranslation + trans;



                            RTemp = RTemp * pointR;


                            RTemp = XNAMatrix.CreateTranslation(trans) * RTemp;
                            if (this.DataContainer.EditMode && Model.Bricks.Count < 3)
                            {
                                RTemp = XNAMatrix.Identity;
                            }

                            
                            R = RTemp * R;
                            if (beingReset)
                            {
                                R = DataContainer.lastConfidentR;
                            }
                            DataContainer.R = R;

                        }
                        if (NormalAligner)
                        {
                            this.DataContainer.backgroundEvaluator.NormalInput.Enqueue(container);
                            NormalAligner = false;
                        }

                    }

                    if (Math.Abs(R.Determinant() - 1) < 0.001f && !skip)
                    {

                        container.onlyRot = onlyRot;
                        NormalCounter++;
                        if (container.ICPMSE < MINICPMSE || !this.REQUIRE_HIGH_QUALITY_RESULT)
                        {
                            container.R = R;
                        }
                        if (container.ICPMSE < MINICPMSE)
                        {
                            
                            this.DataContainer.trackingConfidence = TrackingConfidenceLevel.ICPFULL;
                            trackingLostCount = 0;
                        }
                        else if (container.ICPMSE < 30f)
                        {
                            if (trackingLostCount > 0)
                            {
                                System.Diagnostics.Debug.WriteLine("regained tracking!");
                            }
                            trackingLostCount = 0;
                            resetCount = 0;
                            this.DataContainer.trackingConfidence = TrackingConfidenceLevel.ICPTENTATIVE;
                        }
                        else
                        {
                            this.DataContainer.trackingConfidence = TrackingConfidenceLevel.NONE;
                            trackingLostCount++;
                        }
                        if (trackingLostCount > 100)
                        {
                            //this.Reset();
                            trackingLostCount = 0;
                        }
                        PrevRKnown = true;

                        if(!beingReset)
                            PrevR = R;
                        if (this.DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL)
                        {
                            this.DataContainer.lastConfidentR = R;
                        }

                    }
                    if (this.DataContainer.EditMode)
                    {
                        this.DataContainer.trackingConfidence = TrackingConfidenceLevel.ICPFULL;
                    }


                    

                }
                container.Qi.AddRange(backgroundData);
                int c = 0;
                /*foreach (Point p in Model.points)
                {
                    if (c++ % 10 == 0)
                    {
                        container.Qi.Add(p);
                    }
                }*/
                container.Timings.Add(DateTime.Now);
                File.WriteLine((DateTime.Now - start).Milliseconds.ToString());
                Manager.ProcessingQueues[++container.Stage].Enqueue(container);
                if (DataContainer.Attach && DataContainer.backgroundEvaluator.ModificationInput.Count == 0 && !DataContainer.backgroundEvaluator.ModificationRunning)
                {
                    DataContainer.ModelsWorked = 0;
                    if ((Math.Abs(angleSum) > Math.PI / 6 && DataContainer.ICPMSE < DataContainer.currentMaxMSE) || Model.Bricks.Count < 3)
                    {
                        DataContainer.differentViewCounter++;
                        angleSum = 0;
                        XNAMatrix Rinv = XNAMatrix.Invert(DataContainer.R);

                        /*if (DataContainer.model.Bricks.Count < 3)
                        {
                            foreach (Point p in container.Qi)
                            {

                                Vector3 transformedP = Vector3.Transform(p.position - container.center, Rinv);
                                Point transformed = new Point(transformedP, Vector3.Zero);
                                transformed.brickColorInteger = p.brickColorInteger;
                                transformed.brickColor = p.brickColor;
                                backgroundData.Add(transformed);

                            }
                        }
                        else*/
                        {
                            foreach (Point p in Outliers)
                            {

                                Vector3 transformedP = Vector3.Transform(p.position - container.center, Rinv);
                                Point transformed = new Point(transformedP, Vector3.Zero);
                                transformed.brickColorInteger = p.brickColorInteger;
                                transformed.brickColor = p.brickColor;
                                backgroundData.Add(transformed);

                            }
                        }
                    }
                    if (DataContainer.differentViewCounter > 2 || Model.Bricks.Count < 3 || DataContainer.ICPMSE > 3*Math.Max(DataContainer.currentMaxMSE,25))
                    {
                        DataContainer.backgroundEvaluator.ModificationInput.Enqueue(new List<Point>(backgroundData));
                        
                        backgroundData.Clear();

                        DataContainer.Attach = false;
                        DataContainer.differentViewCounter = 0;
                    }
                }

                DataContainer.comparisonPoints = new List<Point>();
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
                if (DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL && icpCount % 1 == 0)
                {
                    this.DataContainer.backgroundEvaluator.PointsInput.Enqueue(new List<Point>(Outliers));
                }
            }
            File.Close();


        }

        /*public void TryAlign()
        {
            int stage = 7;
            bool work = false;
            bool reported = false;

            while (this.DataContainer.Run)
            {
                PipelineContainer container = null;
                while (container == null && this.DataContainer.Run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (Manager.ProcessingQueues[stage].Count > DataContainer.MINFRAMESINCONTAINER)
                    {
                        Manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.DataContainer.SLEEPTIME);
                        }
                    }
                    else
                    {
                        Thread.Sleep(this.DataContainer.SLEEPTIME);
                    }
                }
                if (container == null)
                {
                    break;
                }
                //container.Timings.Add(DateTime.Now);
                if (this.DataContainer.DeNoiseAndICP || work)
                {
                    
                    XNAMatrix temp;
                    if (this.DataContainer.backgroundEvaluator.NormalOutput.TryDequeue(out temp))
                    {
                        DataContainer.normalRotationAdjustment = temp;
                    }
                    //XNAMatrix onlyRot = container.onlyRot;
                    XNAMatrix rot = DataContainer.prevNormalR;
                    if (DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL || DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPTENTATIVE)
                    {
                        if (DataContainer.lastConfidentR.M11 != 1)
                            rot = DataContainer.lastConfidentR;
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
                    DataContainer.modelNormals[0] = container.modelVectors[0];
                    DataContainer.modelNormals[1] = container.modelVectors[1];
                    DataContainer.modelNormals[2] = container.modelVectors[2];
                    if (container.estimatedVectors[0] != Vector3.Zero)
                    {
                        //modelVectors = container.estimatedVectors;
                    }
                    Vector3[] normals = container.NormalsList;
                    if (DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPFULL || DataContainer.trackingConfidence == TrackingConfidenceLevel.ICPTENTATIVE)
                    {
                        for (int modelV = 0; modelV < 3; modelV++)
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
                        }
                    }


                    for (int modelV = 0; modelV < 3; modelV++)
                    {
                        container.estimatedVectors[modelV] = normals[DataContainer.normalMappings[modelV]];
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
                    result = result * DataContainer.prevNormalR;

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
                    result = result * DataContainer.prevNormalR;
                    DataContainer.prevNormalR = result;
                    //container.normalR = dataContainer.prevNormalR;     
                    container.rawNormalR = result;
                    container.normalR = DataContainer.normalRotationAdjustment * container.rawNormalR;
                    if (!reported && DataContainer.trackingConfidence == TrackingConfidenceLevel.NORMALS)
                    {
                        PrevR = result;//container.normalR;
                        reported = true;
                    }
                }
                container.Timings.Add(DateTime.Now);
                Manager.ProcessingQueues[++container.Stage].Enqueue(container);

                //manager.enqueue(container);
            }


        }*/
        Thread t;
        public void Reset()
        {
            System.Diagnostics.Debug.WriteLine("RESET");
            //DataContainer.lastConfidentR.Translation = Vector3.Zero;
            if (t != null)
            {
                if (t.ThreadState == ThreadState.Running)
                {
                    t.Abort();
                }
            }
            t = new Thread(ResetRoutine);
            t.Start();
            
            /*if (Math.Abs(this.DataContainer.lastConfidentR.Determinant() - 1) < 0.001f)
            {
               
            }
            else
            {
                PrevRKnown = false;
            }*/
            //PrevR = /*XNAMatrix.CreateTranslation(0,0,-15)**/DataContainer.BaseRotation;
            //this.ICPTranslation = Vector3.Zero;
            this.OldCenter = Vector3.Zero;
        }

        private void ResetRoutine()
        {
            beingReset = true;
            int count = 0;
            this.DataContainer.lastConfidentR = this.DataContainer.lastConfidentR;// * XNAMatrix.CreateTranslation(-this.DataContainer.g * 10);
            while (count++ < 1500)
            {
                PrevR = this.DataContainer.lastConfidentR;//XNAMatrix.CreateTranslation(0, 0, -50) * DataContainer.BaseRotation;
                Thread.Sleep(1);
            }
            beingReset = false;

        }


        public Vector3[] KMeans(Vector3[,] normals)
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
        //Stage 5
        public void scanNormals()
        {
            int stage = 5;
            DepthColor[,] dc;

            StreamWriter File;
            DateTime start = DateTime.Now;
            File = new StreamWriter("5 " + stage1Counter++ + ".csv");

            Vector3 up, down, left, right;

            Vector3 position;
            Vector3 normal;
            Vector3 verticalAvg, horizontalAvg;
            while (this.DataContainer.Run)
            {
                //DateTime start = DateTime.Now;
                PipelineContainer container = null;
                while (container == null && this.DataContainer.Run)
                {
                    //container = manager.dequeue(stage);
                    container = null;
                    if (Manager.ProcessingQueues[stage].Count > DataContainer.MINFRAMESINCONTAINER)
                    {
                        Manager.ProcessingQueues[stage].TryDequeue(out container);
                        if (container == null)
                        {
                            Thread.Sleep(this.DataContainer.SLEEPTIME);
                        }

                    }
                    else
                    {
                        Thread.Sleep(this.DataContainer.SLEEPTIME);
                    }

                }
                if (container == null)
                {
                    break;
                }
                start = DateTime.Now;
                container.Timings.Add(DateTime.Now);
                if (this.DataContainer.DeNoiseAndICP)
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
                                //dc[x, y].Color = -normal;

                                if (verticalAvg.Length() > 0 && horizontalAvg.Length() > 0 && normal.Length() > 0)
                                {

                                    normal = -normal;
                                    container.Normals[639 - x, 479 - y] = normal;
                                }
                            }
                        }
                    }
                    //container.NormalsList = this.KMeans(container.Normals);
                }
                File.WriteLine((DateTime.Now - start).Milliseconds.ToString());
                container.Timings.Add(DateTime.Now);
                Manager.ProcessingQueues[++container.Stage].Enqueue(container);


            }
            File.Close();
        }
       
    }
}


