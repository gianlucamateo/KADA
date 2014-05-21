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


namespace KADA
{
    class _3DProcessor
    {
        private ConcurrentQueue<DepthColor[,]> renderQueue, processingQueue;
        private ConcurrentQueue<Vector3> centers;
        private ConcurrentQueue<Microsoft.Xna.Framework.Matrix> rotations;
        public Vector3 oldCenter = Vector3.Zero;
        private readonly float THRESHOLD = 100;
        private KDTreeWrapper brick;
        private static Microsoft.Xna.Framework.Matrix prevR;
        private static bool prevRKnown = false;
        private List<Vector3> qi;
        public double ICPInliers = 0, ICPOutliers = 0, ICPRatio = 0;
        private PCViewer g;

        private int normalCounter=0;

        private const double MINICPRATIO = 2.0;

        public _3DProcessor(ConcurrentQueue<DepthColor[,]> processingQueue, ConcurrentQueue<DepthColor[,]> renderQueue,
            ConcurrentQueue<Vector3> centers, ConcurrentQueue<Microsoft.Xna.Framework.Matrix> rotations, Vector3 offset, PCViewer g)
        {
            this.brick = XYZFileLoader.Reader.readFromFile(offset);
            this.renderQueue = renderQueue;
            this.processingQueue = processingQueue;
            this.rotations = rotations;
            this.centers = centers;
            this.qi = new List<Vector3>();
            this.g = g;

        }

        public void generateCenter(Object dcIn)
        {
            List<Vector3> qi = new List<Vector3>(this.qi);
            DepthColor[,] dc;
            if (processingQueue.TryDequeue(out dc) == false)
                return;

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
            oldCenter = center;

            //ICP
            System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(this.ICP), new ICPDataContainer(dc,center,qi));
            this.centers.Enqueue(center);
            this.renderQueue.Enqueue(dc);
            //this.ICP(center, dc, qi);
        }

        public void ICP(Object input)
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
            Microsoft.Xna.Framework.Matrix R;
            if (!prevRKnown)
            {
                R = Microsoft.Xna.Framework.Matrix.CreateRotationX(0);
            }
            else
            {
                R = prevR;
            }
            Microsoft.Xna.Framework.Matrix RInv = Microsoft.Xna.Framework.Matrix.CreateRotationX(0);
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
                    Microsoft.Xna.Framework.Matrix.Invert(ref R, out RInv);
                }
                foreach (Vector3 v in qi)
                {
                    count++;
                    Vector3 vC = v - center;
                    vC = Microsoft.Xna.Framework.Vector3.Transform(vC, RInv);

                    double[] vArr = new double[] { vC.X, vC.Y, vC.Z };
                    NearestNeighbour<Point> b = brick.NearestNeighbors(vArr, 1, fDistance: THRESHOLD);
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
                    if (b.CurrentDistance < 500)
                    {
                        H.AddInplace(HTemp);
                        if (b.CurrentDistance < 5)
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

                if (this.ICPRatio > MINICPRATIO && i>2)
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
                Microsoft.Xna.Framework.Matrix RTemp = new Microsoft.Xna.Framework.Matrix(
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
                RTemp = Microsoft.Xna.Framework.Matrix.Transpose(RTemp);
                R = Microsoft.Xna.Framework.Matrix.Multiply(R, RTemp);

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
                if(normalCounter>30)
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
        public void kMeans(Vector3[,] normals)
        {
            Bitmap norm = new Bitmap(640, 480);
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
                            norm.SetPixel(x, y, colors[cluster]);
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
            int lessConfidentIndex = 1 - confidentIndex;
            estimatedNormals[lessConfidentIndex] = Vector3.Cross(centroids[confidentIndex], estimatedNormals[2]);
            estimatedNormals[confidentIndex] = Vector3.Cross(estimatedNormals[2], estimatedNormals[lessConfidentIndex]);

            System.Diagnostics.Debug.Write("vector");
            for (int i = 0; i < clusters.Length; i++)
            {
                //centroids[i] *= -1;
                //System.Diagnostics.Debug.Write("{" + centroids[i].X + "," + centroids[i].Y + "," + centroids[i].Z + "},");
                System.Diagnostics.Debug.Write("{" + estimatedNormals[i].X + "," + estimatedNormals[i].Y + "," + estimatedNormals[i].Z + "},");
            }
            norm.Save("normals_clustered.png");
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
            this.kMeans(normals);
            //return bitmap;

        }
    }
}


