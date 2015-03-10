using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.Concurrent;
using Microsoft.Kinect;
using Microsoft.Xna.Framework;

using System.Drawing;
using System.Collections;
using System.Threading;
using Image = System.Drawing.Image;

namespace KADA
{
    class _2DProcessor
    {
        
        private DepthImagePixel[] Background;
        private bool BackgroundReady = false, ColorReady = false;
        private int BackgroundFrameCount = 0;
        private PipelineManager Manager;
        private static Object Semaphor, Normalizer;
        private static DepthImagePixel[][] SingleImages;
        private short[] DepthValues;
        private float AngleFactor = (float)(1.0 / Math.Tan(43.0 / 180.0 / 480 * Math.PI)) * 0.85f;
        
        List<IHistogram> Histograms;
        Bitmap Bitmap = new Bitmap(640, 480);
        private PipelineDataContainer DataContainer;
        private Matrix RGBToYUV, YUVToRGB;


        public _2DProcessor(PipelineManager manager, PipelineDataContainer dataContainer)//(ConcurrentQueue<DepthColor[,]> processingQueue)
        {
            this.RGBToYUV = new Matrix(0.299f, 0.587f, 0.144f, 0f, -0.14713f, -0.28886f, 0.436f, 0f, 0.615f, -0.51499f, -0.10001f, 0f, 0f, 0f, 0f, 1f);
            this.RGBToYUV = Matrix.Transpose(this.RGBToYUV);
            this.YUVToRGB = Matrix.Invert(RGBToYUV);

            this.DataContainer = dataContainer;
            this.Manager = manager;
            SingleImages = new DepthImagePixel[5][];
            this.DepthValues = new short[5];
            Semaphor = new Object();
            Normalizer = new Object();
            Histograms = new List<IHistogram>();

            if (!dataContainer.UseYUV)
            {
                Bitmap red = (Bitmap)Image.FromFile("ressources/histogram/Red_cleaned_filled.png", true);
                IHistogram r = new RGBHistogram(red, 15, 16, XYZFileLoader.BrickColor.RED, dataContainer, new Vector3(123, -20, 118));
                Histograms.Add(r);

                Bitmap green = (Bitmap)Image.FromFile("ressources/histogram/Green_cleaned_filled.png", true);
                IHistogram g = new RGBHistogram(green, 8, 16, XYZFileLoader.BrickColor.GREEN, dataContainer, new Vector3(125, -16, -28));
                Histograms.Add(g);

                Bitmap blue = (Bitmap)Image.FromFile("ressources/histogram/Blue_cleaned_filled.png", true);
                IHistogram b = new RGBHistogram(blue, 8, 16, XYZFileLoader.BrickColor.BLUE, dataContainer, new Vector3(126, 39, -18));
                Histograms.Add(b);

                Bitmap yellow = (Bitmap)Image.FromFile("ressources/histogram/Yellow_cleaned_filled.png", true);
                IHistogram ye = new RGBHistogram(yellow, 12, 16, XYZFileLoader.BrickColor.YELLOW, dataContainer, new Vector3(126, -65, 41));
                Histograms.Add(ye);
            }
            else
            {
                Bitmap blacklist = (Bitmap)Image.FromFile("ressources/histogram/YUV_blacklist.bmp", true);
                Bitmap red = (Bitmap)Image.FromFile("ressources/histogram/YUV_red.bmp", true);
                Bitmap green = (Bitmap)Image.FromFile("ressources/histogram/YUV_green.bmp", true);
                Bitmap blue = (Bitmap)Image.FromFile("ressources/histogram/YUV_blue.bmp", true);
                Bitmap yellow = (Bitmap)Image.FromFile("ressources/histogram/YUV_yellow.bmp", true);               
                IHistogram g = new YUVHistogram(green, blacklist, XYZFileLoader.BrickColor.GREEN);
                Histograms.Add(g);
                IHistogram r = new YUVHistogram(red, blacklist, XYZFileLoader.BrickColor.RED);
                Histograms.Add(r);
                IHistogram b = new YUVHistogram(blue, blacklist, XYZFileLoader.BrickColor.BLUE);
                Histograms.Add(b);
                IHistogram y = new YUVHistogram(yellow, blacklist, XYZFileLoader.BrickColor.YELLOW);
                //Histograms.Add(y);
            }

            List<Thread> stage1 = new List<Thread>();

            for (int i = 0; i < 3; i++)
            {
                Thread x = new Thread(new ThreadStart(() => UpdateDepthData()));
                x.Start();
                stage1.Add(x);
            }


            List<Thread> stage2 = new List<Thread>();

            for (int i = 0; i < 1; i++)
            {
                Thread x = new Thread(new ThreadStart(() => EliminateColor()));
                x.Start();
                stage2.Add(x);
            }

            List<Thread> stage3 = new List<Thread>();

            for (int i = 0; i < 4; i++)
            {
                Thread x = new Thread(new ThreadStart(() => DeNoise()));
                x.Start();
                stage3.Add(x);
            }


        }

        // Stage 1
        private void UpdateDepthData()
        {
            DepthImagePixel[] background = null;
            int stage = 1;
            while (this.DataContainer.Run)
            {
                PipelineContainer container = null;
                container = PollContainer(stage, container);
                if (container == null)
                {
                    break;
                }
                // generate or get background
                DepthImagePixel[] dPixels = container.DepthPixels;
                
                DepthColor[,] depth = container.DC;
                ColorImagePoint[] colorPoints = container.ColorPoints;
                byte[] colorPixels = container.ColorPixels;
                if (this.DataContainer.GenerateBackground && background == null)
                {
                    this.GenerateBackground(dPixels);
                }
                if (background == null)
                {
                    background = GetBackground();
                }

                CullBackground(background, dPixels, depth, colorPoints, colorPixels);


                AddToQueue(container);
            }
        }

        private void AddToQueue(PipelineContainer container)
        {
            //container.Timings.Add(DateTime.Now);

            Manager.ProcessingQueues[++container.Stage].Enqueue(container);
        }

        private void CullBackground(DepthImagePixel[] background, DepthImagePixel[] dPixels, DepthColor[,] depth, ColorImagePoint[] colorPoints, byte[] colorPixels)
        {
            int baseindex;
            for (int i = 0; i < 640 * 480; i++)
            {
                int x = i % 640;
                int y = 480 - (i / 640) - 1;
                if ((colorPoints[i].X >= 0 && colorPoints[i].X < 640 && colorPoints[i].Y >= 0 && colorPoints[i].Y < 480))
                {

                    depth[x, y].Depth = dPixels[i].Depth;
                    float scale = -dPixels[i].Depth / AngleFactor;
                    depth[x, y].Position.X = (x - 320) * scale;
                    depth[x, y].Position.Y = -(y - 240) * scale;
                    depth[x, y].Position.Z = -dPixels[i].Depth;
                    if (background != null)
                    {
                        bool drop = dPixels[i].Depth > background[i].Depth;
                        if (drop)
                        {
                            depth[x, y].Position *= 0;
                            depth[x, y].Depth = 0;
                        }
                    }

                    ColorImagePoint p = colorPoints[i];
                    baseindex = (p.X + p.Y * 640) * 4;
                    Vector3 c = depth[x, y].Color;



                    c.X = colorPixels[baseindex + 2];
                    c.Y = colorPixels[baseindex + 1];
                    c.Z = colorPixels[baseindex];

                    c = c / 255f;
                    if (DataContainer.UseYUV)
                    {
                        c = Vector3.Transform(c, RGBToYUV);
                        c.X = 0.5f;

                        //c = Vector3.Transform(c, YUVToRGB);
                    }
                    //System.Diagnostics.Debug.WriteLine(c);
                    depth[x, y].Color = c;

                    depth[x, y].UpToDate = true;
                }

            }
        }

        private PipelineContainer PollContainer(int stage, PipelineContainer container)
        {
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
            //if(container!=null)
                //container.Timings.Add(DateTime.Now);
            return container;
        }

        public bool GenerateBackground(DepthImagePixel[] depth)
        {
            if (this.BackgroundReady)
            {
                return true;
            }
            lock (Semaphor)
            {
                if (BackgroundFrameCount < SingleImages.Length)
                {
                    SingleImages[BackgroundFrameCount++] = depth;
                    return false;
                }
                else
                {
                    Background = new DepthImagePixel[SingleImages[0].Length];
                    for (int i = 0; i < SingleImages[0].Length; i++)
                    {
                        for (int o = 0; o < 5; o++)
                        {
                            DepthValues[o] = SingleImages[o][i].Depth;
                        }
                        //Array.Sort(depthValues);
                        short localDepth = (short)(DepthValues[1] + DepthValues[2] + DepthValues[3]);
                        localDepth /= 3;
                        localDepth = (short)(localDepth * 0.988f);
                        Background[i].Depth = (short)(localDepth);
                    }
                }
                this.DataContainer.GenerateBackground = false;
                this.BackgroundReady = true;
                return true;
            }
        }

        public DepthImagePixel[] GetBackground()
        {
            if (this.BackgroundReady)
                return this.Background;
            else
                return null;
        }

        public bool IsBackgroundReady()
        {
            return this.BackgroundReady;
        }

        public bool IsColorReady()
        {
            return this.ColorReady;
        }

        // Stage 2
        public void EliminateColor()
        {
            int stage = 2;
            bool work = false;
            while (this.DataContainer.Run)
            {
                PipelineContainer container = null;

                container = PollContainer(stage, container);

                if (container == null)
                {
                    break;
                }
                if (this.DataContainer.DeNoiseAndICP || work)
                {
                    DepthColor[,] dc = container.DC;
                    for (int x = 0; x < dc.GetLength(0); x++)
                    {
                        for (int y = 0; y < dc.GetLength(1); y++)
                        {
                            CullColors(dc, x, y);
                        }
                    }
                }
                AddToQueue(container);
                
            }


        }

        private void CullColors(DepthColor[,] dc, int x, int y)
        {
            DepthColor pixel = dc[x, y];
            Vector3 color = pixel.Color * 256;

            if (pixel.Position.Z != 0)
            {
                int brickColorInteger = 1;
                int maxval = 0;

                foreach (IHistogram h in this.Histograms)
                {
                    int val = h.GetValue(color);
                    if (maxval < val)
                    {
                        maxval = val;
                    }
                    if (val == 1)
                    {
                        brickColorInteger *= (int)h.GetColor();
                    }
                }

                if (maxval == 0 || (color.X + color.Y + color.Z < 150 && !DataContainer.UseYUV))
                {
                    pixel.Position.Z = 0;
                    pixel.Depth = 0;
                    pixel.BrickColorInteger = brickColorInteger;

                }
                dc[x, y] = pixel;
            }
        }

        // Stage 3
        public void DeNoise()
        {
            bool work = false;
            int stage = 3;
            int width = 640, height = 480;
            int[,] grid = new int[width, height];
            int[,] outputGrid = new int[width, height];
            while (this.DataContainer.Run)
            {
                PipelineContainer container = null;
                container = PollContainer(stage, container);
                if (container == null)
                {
                    break;
                }
                if (this.DataContainer.DeNoiseAndICP || work)
                {
                    work = true;
                    DepthColor[,] dc = container.DC;
                    //initialize
                    for (int x = 0; x < width; x++)
                    {
                        for (int y = 0; y < height; y++)
                        {
                            grid[x, y] = 0;
                        }
                    }
                    for (int x = 0; x < width; x++)
                    {
                        for (int y = 0; y < height; y++)
                        {
                            if (dc[x, y].Depth > 0)
                            {
                                grid[x, y] = 1;
                            }
                        }
                    }


                    for (int i = 0; i < 2; i++)
                    {
                        Erode(width, height, grid, outputGrid);
                    }
                    int[,] lastGrid = null;
                    for (int i = 0; i < 3; i++)
                    {
                        lastGrid = Dilate(width, height, grid, outputGrid, lastGrid);
                    }

                    for (int x = 1; x < width - 1; x++)
                    {
                        for (int y = 1; y < height - 1; y++)
                        {
                            if (grid[x, y] > 0)
                            {
                                dc[x, y].Depth = 1;
                                //output.SetPixel(x, y, Color.Black);
                            }
                            else
                            {
                                dc[x, y].Depth = 0;
                                dc[x, y].Position.Z = 0;
                            }
                        }
                    }
                }
                //manager.enqueue(container);
                AddToQueue(container);
                
            }

        }

        private static int[,] Dilate(int width, int height, int[,] grid, int[,] outputGrid, int[,] lastGrid)
        {
            for (int x = 1; x < width - 1; x++)
            {
                for (int y = 1; y < height - 1; y++)
                {
                    int sum = outputGrid[x - 1, y - 1] + outputGrid[x - 1, y + 1]+outputGrid[x + 1, y - 1]+outputGrid[x + 1, y + 1]+outputGrid[x, y + 1]+outputGrid[x, y - 1] +
                        outputGrid[x + 1, y] + outputGrid[x - 1, y];

                    grid[x,y] = sum > 0 ?  1 : 0;
                    /*if (outputGrid[x - 1, y - 1] == true)
                    {
                        grid[x, y] = true;
                    }
                    if (outputGrid[x - 1, y + 1] == true)
                    {
                        grid[x, y] = true;
                    }
                    if (outputGrid[x + 1, y - 1] == true)
                    {
                        grid[x, y] = true;
                    }
                    if (outputGrid[x + 1, y + 1] == true)
                    {
                        grid[x, y] = true;
                    }

                    if (outputGrid[x, y + 1] == true)
                    {
                        grid[x, y] = true;
                    }

                    if (outputGrid[x, y - 1] == true)
                    {
                        grid[x, y] = true;
                    }

                    if (outputGrid[x + 1, y] == true)
                    {
                        grid[x, y] = true;
                    }

                    if (outputGrid[x - 1, y] == true)
                    {
                        grid[x, y] = true;
                    }*/
                    
                }
            }

            lastGrid = outputGrid;
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    outputGrid[x, y] = grid[x, y];
                }
            }
            return lastGrid;
        }

        public static void Erode(int width, int height, int[,] grid, int[,] outputGrid)
        {
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    outputGrid[x, y] = 0;
                }
            }

            for (int x = 1; x < width - 1; x++)
            {
                for (int y = 1; y < height - 1; y++)
                {
                    int diagonalContact = grid[x - 1, y - 1] + grid[x - 1, y + 1] + grid[x + 1, y - 1] + grid[x + 1, y + 1];
                    /*int diagonalContact = 0;
                    if (grid[x - 1, y - 1] == true)
                    {
                        diagonalContact++;
                    }

                    if (grid[x - 1, y + 1] == true)
                    {
                        diagonalContact++;
                    }

                    if (grid[x + 1, y - 1] == true)
                    {
                        diagonalContact++;
                    }

                    if (grid[x + 1, y + 1] == true)
                    {
                        diagonalContact++;
                    }
                    */

                    if (grid[x, y]>0 && diagonalContact > 3) // && directContact > 3 &&
                    {
                        outputGrid[x, y] = 1;
                    }
                }
            }
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    grid[x, y] = outputGrid[x, y];
                }
            }
        }

    }
}
