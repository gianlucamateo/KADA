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

namespace KADA
{
    class ImageProcessor
    {
        private ConcurrentQueue<DepthColor[,]> renderQueue;
        private DepthImagePixel[] background;
        private bool backgroundReady = false;
        private int backgroundFrameCount = 0;
        private static Object Semaphor;
        private static DepthImagePixel[][] singleImages;
        private short[] depthValues;
        

        public ImageProcessor(ConcurrentQueue<DepthColor[,]> renderQueue)
        {
            singleImages = new DepthImagePixel[5][];
            this.renderQueue = renderQueue;
            this.depthValues = new short[5];
            Semaphor = new Object();
            
        }

        public void saveColorsToFile(DepthColor[,] reduced)
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
                if (colorSum>0)
                    color /= colorSum;
                image.SetPixel((int)(1000 * color.Y), (int)(1000 * color.Z), System.Drawing.Color.FromArgb((int)(255 * color.X), (int)(255 * color.Y), (int)(255 * color.Z)));
            }

            image.Save("Colors.png");
            
        }

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
        public bool ready()
        {
            return this.backgroundReady;
        }

        public void eliminateColor(DepthColor[,] dc)
        {
            lock (Semaphor)
            {
                for (int x = 0; x < dc.GetLength(0); x++)
                {
                    for (int y = 0; y < dc.GetLength(1); y++)
                    {
                        DepthColor pixel = dc[x, y];
                        Vector3 color = pixel.Color;
                        if (pixel.Depth != 0)
                        {
                            color.Normalize();
                            float saturation = (Math.Max(Math.Max(color.X, color.Y), color.Z) - Math.Min(Math.Min(color.X, color.Y), color.Z)) / Math.Max(Math.Max(color.X, color.Y), color.Z);
                            bool chanceRed = (color.X / (color.Y) > 2) && (color.X / (color.Z)) > 2 && (Math.Abs(color.Y-color.Z))<0.2f;
                            bool chanceGreen = (color.Y / (color.X) > 1);// && (color.Y / (color.Z)) > 2 && (Math.Abs(color.X - color.Z)) < 0.2f;
                            /*if (saturation < 0.5f)//||!(chanceGreen))
                            {
                                pixel.Depth = 0;
                                dc[x, y] = pixel;
                            }*/
                            if (chanceGreen)
                            {
                                chanceGreen = true;
                            }
                            /*if (chanceRed)
                            {
                                dc[x, y].Color.X = 1;
                                dc[x, y].Color.Y = 1;
                                dc[x, y].Color.Z = 1;
                            }*/
                        }
                    }
                }
            }
            this.renderQueue.Enqueue(dc);
        }

    }
}
