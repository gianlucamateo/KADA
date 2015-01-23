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
        public ConcurrentQueue<Point> input;
        public Vector3 center;
        public XNAMatrix RInv;
        public XNAMatrix onlyRot;
        public Vector3 ICPTranslation;
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

        Point p, firstGuess;
        Vector3 transformedNormal;

        public ICPWorker(int number, PipelineDataContainer dataContainer)
        {
            this.ICPOutliers = 0;
            this.ICPInliers = 0;
            this.ICPTranslation = Vector3.Zero;
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
            Vector3 v, vC;
            Matrix tmA = new Matrix(6, 6);
            Vector tempB = new Vector(6);
            DateTime start = DateTime.Now;
            int count = 0;
            int i = 0;
            double[] vArr = new double[3];
            double maxDistance = _3DProcessor.THRESHOLD;
            NearestNeighbour<Point> neighbour;
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

                if (container == null)
                {
                    break;
                }
                i = 0;
                count++;
                vC = v - center;
                vC = Microsoft.Xna.Framework.Vector3.Transform(vC, RInv);
                vArr[0] = vC.X;
                vArr[1] = vC.Y;
                vArr[2] = vC.Z;
                //maxDistance = _3DProcessor.THRESHOLD;
                //if (container.ICPRatio > 0.3f)
                //{
                //    maxDistance = _3DProcessor.MAX_INLIERDISTANCE;
                //}

                neighbour = brickWrapper.NearestNeighbors(vArr, 5, fDistance: maxDistance);
                neighbour.MoveNext();

                transformedNormal = Vector3.Zero;
                firstGuess = neighbour.Current;
                double distance = 0f;


                bool found = true;
                if (container.ICPRatio > 0.2f)
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
                        continue;
                    }
                }

                p = neighbour.Current;
                transformedNormal = Vector3.Transform(p.normal, onlyRot);
                //transformedNormal = Vector3.Transform(p.normal, onlyRot);
                if (p.normal == Vector3.Zero)
                {
                    //p = firstGuess;

                    container.ICPOutliers++;
                    //return;
                    continue;
                }

                vC /= dataContainer.model.radius;
                p.position /= dataContainer.model.radius;

                HTemp[0, 0] = vC.X * p.position.X;
                HTemp[0, 1] = vC.Y * p.position.X;
                HTemp[0, 2] = vC.Z * p.position.X;
                HTemp[1, 0] = vC.X * p.position.Y;
                HTemp[1, 1] = vC.Y * p.position.Y;
                HTemp[1, 2] = vC.Z * p.position.Y;
                HTemp[2, 0] = vC.X * p.position.Z;
                HTemp[2, 1] = vC.Y * p.position.Z;
                HTemp[2, 2] = vC.Z * p.position.Z;



                float weight = Math.Abs(Vector3.Dot(transformedNormal, point.normal));
                weight = (1 - weight);//(float)Math.Log((1-weight)*2+1)+0.05f;//
                foreach (BrickColor bc in possibleColors)
                {
                    int number = (int)bc;
                    if ((p.brickColorInteger / number) * number == p.brickColorInteger)
                    {
                        if (point.brickColor == bc)
                        {
                            weight *= 10f;
                            break;
                        }
                    }
                }

                Vector3 pos = p.position;
                //pos /= 20;
                //vC /= 20;
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
                //System.Diagnostics.Debug.WriteLine(weight);
                tmA.MultiplyInplace(weight);

                //double[] tempB = new double[6];

                float pMinqTimesN = Vector3.Dot(pos - vC, n);

                tempB[0] = pMinqTimesN * c.X;
                tempB[1] = pMinqTimesN * c.Y;
                tempB[2] = pMinqTimesN * c.Z;
                tempB[3] = pMinqTimesN * n.X;
                tempB[4] = pMinqTimesN * n.Y;
                tempB[5] = pMinqTimesN * n.Z;


                tempB.MultiplyInplace(weight);

                HTemp.MultiplyInplace(weight);


                totalWeight += weight;
                H.AddInplace(HTemp);

                A = A.Add(tmA);
                B = B.Subtract(tempB);
                if (neighbour.CurrentDistance < _3DProcessor.MAX_INLIERDISTANCE)
                {
                    this.ICPInliers++;
                }
                else
                {
                    this.ICPOutliers++;
                }
            }
        }
    }
}
