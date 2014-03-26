using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.Concurrent;
using Microsoft.Kinect;
using Microsoft.Xna.Framework;
using System.Windows.Controls;
using System.Windows.Shapes;
using System.Windows;
using System.Drawing;
using System.Collections;
using System.Threading;
using Image = System.Drawing.Image;

namespace KADA
{
    class ImageProcessor
    {
        private ConcurrentQueue<DepthColor[,]> renderQueue;
        private DepthImagePixel[] background;
        private bool backgroundReady = false, colorReady = false;
        private bool readyForNormals = false;
        private int backgroundFrameCount = 0;
        private static Object Semaphor, Normalizer;
        private static DepthImagePixel[][] singleImages;
        private short[] depthValues;
        private readonly float MAXCOLROSPACEDISTANCE = 50;
        private Vector3[] colors = new Vector3[4];
        List<Histogram> Histograms;
        Bitmap bitmap = new Bitmap(640, 480);
        public static ManualResetEvent resetEvent = new ManualResetEvent(false);


        public ImageProcessor(ConcurrentQueue<DepthColor[,]> renderQueue)
        {
            singleImages = new DepthImagePixel[5][];
            this.renderQueue = renderQueue;
            this.depthValues = new short[5];
            Semaphor = new Object();
            Normalizer = new Object();
            Histograms = new List<Histogram>();

            Bitmap red = (Bitmap)Image.FromFile("../../ressources/histogram/Red_cleaned_filled.png", true);
            Histogram r = new Histogram(red, 20, 16);
            Histograms.Add(r);

            Bitmap green = (Bitmap)Image.FromFile("../../ressources/histogram/Green_cleaned_filled.png", true);
            Histogram g = new Histogram(green, 15, 16);
            Histograms.Add(g);

            Bitmap blue = (Bitmap)Image.FromFile("../../ressources/histogram/Blue_cleaned_filled.png", true);
            Histogram b = new Histogram(blue, 15, 16);
            Histograms.Add(b);

            Bitmap yellow = (Bitmap)Image.FromFile("../../ressources/histogram/Yellow_cleaned_filled.png", true);
            Histogram ye = new Histogram(yellow, 18, 16);
            Histograms.Add(ye);

        }

        /*public void saveColorsToFile(DepthColor[,] reduced)
        {

            Bitmap image = new Bitmap(1000, 1000);
            foreach (DepthColor d in reduced)
            {
                if (d.Depth == 0)
                {
                    continue;
                }
                Vector3 color = d.Color;
                float colorSum = color.X + color.Y + color.Z;
                if (colorSum > 0)
                    color /= colorSum;
                image.SetPixel((int)(1000 * color.Y), (int)(1000 * color.Z), System.Drawing.Color.FromArgb((int)(255 * color.X), (int)(255 * color.Y), (int)(255 * color.Z)));
            }

            kMeans(reduced);

            //image.Save("Colors.png");
            this.colorReady = true;

        }

        public void kMeans(DepthColor[,] dc)
        {
            Vector3 red = new Vector3(250, 15,30);
            Vector3 green = new Vector3(90, 200,115);
            Vector3 blue = new Vector3(70, 100,220);
            Vector3 yellow = new Vector3(210, 140, 50);
            //Vector3 noise = new Vector2(300, 300);

            Vector3[] centroids = { red, green, blue, yellow};

            List<Vector3>[] clusters = new List<Vector3>[4];
            
            for(int i = 0; i<clusters.Length;i++)
            {
                centroids[i] = this.sumNormalize(centroids[i]);
                clusters[i] = new List<Vector3>();
            }

            for (int count = 0; count < 5; count++)
            {
                for (int x = 0; x < 640; x++)
                {
                    for (int y = 0; y < 480; y++)
                    {
                        if (dc[x, y].Position.Z != 0)
                        {
                            Vector3 position = sumNormalize(dc[x,y].Color);
                            float distance = float.MaxValue;
                            int cluster = int.MaxValue;
                            for (int i = 0; i < clusters.Length; i++)
                            {
                                float temp_distance = 0;
                                Vector3.Distance(ref position, ref centroids[i], out temp_distance);
                                if (temp_distance < distance && temp_distance < MAXCOLROSPACEDISTANCE)
                                {
                                    cluster = i;
                                    distance = temp_distance;
                                }

                            }
                            if (cluster < clusters.Length)
                                clusters[cluster].Add(position);
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
                    center /= clusters[i].Count;
                    centroids[i] = center;
                    if (i == 3)
                    {
                        i++;
                    }

                }
                for (int i = 0; i < clusters.Length; i++)
                {
                    clusters[i].Clear();
                }
            }
            for (int i = 0; i < clusters.Length; i++)
            {
                this.colors[i] = centroids[i];
            }
        }*/

        public bool GenerateBackground(DepthImagePixel[] depth)
        {
            lock (Semaphor)
            {
                if (backgroundFrameCount < singleImages.Length)
                {
                    singleImages[backgroundFrameCount++] = depth;
                    return false;
                }
                else
                {
                    background = new DepthImagePixel[singleImages[0].Length];
                    for (int i = 0; i < singleImages[0].Length; i++)
                    {
                        for (int o = 0; o < 5; o++)
                        {
                            depthValues[o] = singleImages[o][i].Depth;
                        }
                        //Array.Sort(depthValues);
                        short localDepth = (short)(depthValues[1] + depthValues[2] + depthValues[3]);
                        localDepth /= 3;
                        localDepth = (short)(localDepth * 0.988f);
                        background[i].Depth = (short)(localDepth);
                    }
                }
                this.backgroundReady = true;
                return true;
            }
        }

        public DepthImagePixel[] getBackground()
        {
            if (this.backgroundReady)
                return this.background;
            else
                return null;
        }
        public bool BackgroundReady()
        {
            return this.backgroundReady;
        }
        public bool ColorReady()
        {
            return this.colorReady;
        }

        //returns a sum-normalized vector, components ranging from 0-255; 
        private Vector3 sumNormalize(Vector3 vec)
        {
            //float sum = vec.X + vec.Y + vec.Z;
            //vec /= sum;
            vec.Normalize();
            vec *= 255;
            return vec;
        }

        /*public void eliminateColor(Object dcIn)
        {
            DepthColor[,] dc = (DepthColor[,])dcIn;
            lock (Semaphor)
            {
               
                Vector2 gToB = new Vector2(0,0);
                for (int x = 0; x < dc.GetLength(0); x++)
                {
                    for (int y = 0; y < dc.GetLength(1); y++)
                    {
                        DepthColor pixel = dc[x, y];
                        Vector3 color = sumNormalize(pixel.Color);
                        
                        if (pixel.Position.Z != 0)
                        {                            
                            float saturation = (Math.Max(Math.Max(color.X, color.Y), color.Z) - Math.Min(Math.Min(color.X, color.Y), color.Z)) / Math.Max(Math.Max(color.X, color.Y), color.Z);
                            //color.Normalize();                                                      
                            bool chanceRed = (color.X / (color.Y) > 2) && (color.X / (color.Z)) > 2 && (Math.Abs(color.Y - color.Z)) < 0.2f;
                            bool chanceGreen = (color.Y / (color.X) > 1);// && (color.Y / (color.Z)) > 2 && (Math.Abs(color.X - color.Z)) < 0.2f;
                            bool accept = false;
                            float distance;
                            float bonus = 0;

                            
                            for (int possibleColor = 0; possibleColor < this.colors.Length; possibleColor++)
                            {
                                Vector3.Distance(ref color, ref this.colors[possibleColor], out distance);
                                if (distance < MAXCOLROSPACEDISTANCE+bonus)
                                    accept = true;
                            }
                            
                            if (!accept)//|| saturation < 0.8)//||!(chanceGreen))
                            {
                                pixel.Position *= 0;
                                pixel.Depth = 0;
                                dc[x, y] = pixel;
                            }
                            if (chanceGreen)
                            {
                                chanceGreen = true;
                            }
                            
                        }
                    }
                }
            }
            readyForNormals = false;
            Bitmap bitmap = new Bitmap(640, 480);
            //DepthColor[,] dc = (DepthColor[,])dcIn;

            Vector3 up, down, left, right;
            Vector3 zero = new Vector3(0, 0, 0);
            Vector3 position;
            Vector3 normal;
            Vector3 verticalAvg, horizontalAvg;
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
                        normal = Vector3.Cross(verticalAvg, horizontalAvg);
                        normal.Normalize();
                        if (verticalAvg.Length() > 0 && horizontalAvg.Length() > 0)
                        {
                            //bitmap.SetPixel(x, 479 - y, System.Drawing.Color.FromArgb(254, 127 + (int)(normal.X * 127), 127 + (int)(normal.Y * 127), 127 + (int)(normal.Z * 127)));
                        }
                    }
                }
            }
            this.renderQueue.Enqueue(dc);
            //bitmap.Save("normals.png");
            resetEvent.Set();
        }*/
        public void eliminateColor(Object dcIn)
        {
            DepthColor[,] dc = (DepthColor[,])dcIn;
            lock (Semaphor)
            {                
                for (int x = 0; x < dc.GetLength(0); x++)
                {
                    for (int y = 0; y < dc.GetLength(1); y++)
                    {
                        DepthColor pixel = dc[x, y];
                        Vector3 color = pixel.Color*256;

                        if (pixel.Position.Z != 0)
                        {
                            int maxval = 0;
                            foreach (Histogram h in this.Histograms)
                            {
                                int val = h.getValue(color);
                                if (maxval < val)
                                {
                                    maxval = val;
                                }
                            }
                            if (maxval == 0)
                            {
                                pixel.Position.Z = 0;
                                pixel.Depth = 0;
                                
                            }
                            dc[x, y] = pixel;
                        }
                    }
                }
            }            
            this.renderQueue.Enqueue(dc);
            //bitmap.Save("normals.png");
            resetEvent.Set();
        }

        public void scanNormals(Object dcIn)
        {
            lock (Normalizer)
            {
                readyForNormals = false;
                bitmap = new Bitmap(640, 480);
                DepthColor[,] dc = (DepthColor[,])dcIn;

                Vector3 up, down, left, right;
                Vector3 zero = new Vector3(0, 0, 0);
                Vector3 position;
                Vector3 normal;
                Vector3 verticalAvg, horizontalAvg;
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
                            normal = Vector3.Cross(verticalAvg, horizontalAvg);
                            normal.Normalize();
                            if (verticalAvg.Length() > 0 && horizontalAvg.Length() > 0)
                            {
                                bitmap.SetPixel(x, 479 - y, System.Drawing.Color.FromArgb(254, 127 + (int)(normal.X * 127), 127 + (int)(normal.Y * 127), 127 + (int)(normal.Z * 127)));
                            }
                        }
                    }
                }
               
                bitmap.Save("normals.png");
                //return bitmap;

            }
        }

    }
}
