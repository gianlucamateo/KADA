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
        private ConcurrentQueue<DepthColor[,]> processingQueue;
        private DepthImagePixel[] background;
        private bool backgroundReady = false, colorReady = false;

        public static List<Vector2> edgeCoordinates;

        private int backgroundFrameCount = 0;
        private PipelineManager manager;
        private static Object Semaphor, Normalizer;
        private static DepthImagePixel[][] singleImages;
        private short[] depthValues;
        private float angleFactor = (float)(1.0 / Math.Tan(43.0 / 180.0 / 480 * Math.PI)) * 0.85f;
        //private Vector3[] colors = new Vector3[4];
        List<IHistogram> Histograms;
        Bitmap bitmap = new Bitmap(640, 480);
        private PipelineDataContainer dataContainer;
        public static ManualResetEvent resetEvent = new ManualResetEvent(false);
        private Matrix RGBToYUV, YUVToRGB;


        public _2DProcessor(PipelineManager manager, PipelineDataContainer dataContainer)//(ConcurrentQueue<DepthColor[,]> processingQueue)
        {
            this.RGBToYUV = new Matrix(0.299f, 0.587f, 0.144f, 0f, -0.14713f, -0.28886f, 0.436f, 0f, 0.615f, -0.51499f, -0.10001f, 0f, 0f, 0f, 0f, 1f);
            this.RGBToYUV = Matrix.Transpose(this.RGBToYUV);
            this.YUVToRGB = Matrix.Invert(RGBToYUV);

            this.dataContainer = dataContainer;
            this.manager = manager;
            singleImages = new DepthImagePixel[5][];
            this.depthValues = new short[5];
            Semaphor = new Object();
            Normalizer = new Object();
            Histograms = new List<IHistogram>();

            if (!dataContainer.useYUV)
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

            List<Thread> Stage1 = new List<Thread>();

            for (int i = 0; i < 3; i++)
            {
                Thread x = new Thread(new ThreadStart(() => UpdateDepthData()));
                x.Start();
                Stage1.Add(x);
            }


            List<Thread> Stage2 = new List<Thread>();

            for (int i = 0; i < 1; i++)
            {
                Thread x = new Thread(new ThreadStart(() => eliminateColor()));
                x.Start();
                Stage2.Add(x);
            }

            List<Thread> Stage3 = new List<Thread>();

            for (int i = 0; i < 2; i++)
            {
                Thread x = new Thread(new ThreadStart(() => deNoise()));
                x.Start();
                Stage3.Add(x);
            }


        }

        //Stage 1
        private void UpdateDepthData()
        {
            DepthImagePixel[] background = null;
            int stage = 1;
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
                DepthImagePixel[] dPixels = container.depthPixels;
                int baseindex;
                DepthColor[,] depth = container.dc;
                ColorImagePoint[] colorPoints = container.colorPoints;
                byte[] colorPixels = container.colorPixels;
                if (this.dataContainer.generateBackground && background == null)
                {
                    this.GenerateBackground(dPixels);
                }
                if (background == null)
                {
                    background = getBackground();
                }
                for (int i = 0; i < 640 * 480; i++)
                {
                    int x = i % 640;
                    int y = 480 - (i / 640) - 1;
                    if ((colorPoints[i].X >= 0 && colorPoints[i].X < 640 && colorPoints[i].Y >= 0 && colorPoints[i].Y < 480))
                    {

                        depth[x, y].Depth = dPixels[i].Depth;
                        float scale = -dPixels[i].Depth / angleFactor;
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
                        if (dataContainer.useYUV)
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
                //manager.enqueue(container);
                container.timings.Add(DateTime.Now);

                manager.processingQueues[++container.stage].Enqueue(container);
            }
        }



        public bool GenerateBackground(DepthImagePixel[] depth)
        {
            if (this.backgroundReady)
            {
                return true;
            }
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
                this.dataContainer.generateBackground = false;
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




        //Stage 2
        public void eliminateColor()
        {
            int stage = 2;
            bool work = false;
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

                    DepthColor[,] dc = container.dc;
                    for (int x = 0; x < dc.GetLength(0); x++)
                    {
                        for (int y = 0; y < dc.GetLength(1); y++)
                        {
                            DepthColor pixel = dc[x, y];
                            Vector3 color = pixel.Color * 256;

                            if (pixel.Position.Z != 0)
                            {
                                int brickColorInteger = 1;
                                int maxval = 0;

                                foreach (IHistogram h in this.Histograms)
                                {
                                    int val = h.getValue(color);
                                    if (maxval < val)
                                    {
                                        maxval = val;
                                    }
                                    if (val == 1)
                                    {
                                        brickColorInteger *= (int)h.getColor();
                                    }
                                }

                                if (maxval == 0 || (color.X + color.Y + color.Z < 150&&!dataContainer.useYUV))
                                {
                                    pixel.Position.Z = 0;
                                    pixel.Depth = 0;
                                    pixel.BrickColorInteger = brickColorInteger;

                                }
                                dc[x, y] = pixel;
                            }
                        }
                    }
                }
                container.timings.Add(DateTime.Now);
                manager.processingQueues[++container.stage].Enqueue(container);
                //manager.enqueue(container);
            }


        }



        //Stage 3
        public void deNoise()
        {
            bool work = false;
            int stage = 3;
            int width = 640, height = 480;
            bool[,] grid = new bool[width, height];
            bool[,] outputGrid = new bool[width, height];
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
                    work = true;
                    DepthColor[,] dc = container.dc;

                    //int width = dc.GetLength(0), height = dc.GetLength(1);

                    //bool[,] grid = new bool[width, height];
                    //bool[,] outputGrid = null;

                    for (int x = 0; x < width; x++)
                    {
                        for (int y = 0; y < height; y++)
                        {
                            grid[x, y] = false;
                        }
                    }

                    for (int x = 0; x < width; x++)
                    {
                        for (int y = 0; y < height; y++)
                        {
                            if (dc[x, y].Depth > 0)
                            {
                                grid[x, y] = true;
                            }
                        }
                    }
                    for (int i = 0; i < 2; i++)
                    {

                        for (int x = 0; x < width; x++)
                        {
                            for (int y = 0; y < height; y++)
                            {
                                outputGrid[x, y] = false;
                            }
                        }

                        for (int x = 1; x < width - 1; x++)
                        {
                            for (int y = 1; y < height - 1; y++)
                            {
                                int diagonalContact = 0;
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


                                if (grid[x, y] && diagonalContact > 3) // && directContact > 3 &&
                                {
                                    outputGrid[x, y] = true;
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
                    bool[,] lastGrid = null;
                    for (int i = 0; i < 3; i++)
                    {

                        for (int x = 1; x < width - 1; x++)
                        {
                            for (int y = 1; y < height - 1; y++)
                            {
                                if (outputGrid[x - 1, y - 1] == true)
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
                                }
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

                    }

                    for (int x = 1; x < width - 1; x++)
                    {
                        for (int y = 1; y < height - 1; y++)
                        {
                            if (grid[x, y] == true)
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
                container.timings.Add(DateTime.Now);
                manager.processingQueues[++container.stage].Enqueue(container);
            }

        }

    }
}
