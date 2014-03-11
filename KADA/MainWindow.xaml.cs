using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Runtime.InteropServices;

using System.IO;
using Microsoft.Kinect;
using System.Threading;
using System.Threading.Tasks;
using System.Drawing;
using System.Drawing.Imaging;




namespace KADA
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinect;

        private WriteableBitmap imageBitmap;
        private WriteableBitmap depthBitmap;
        private Bitmap image = new Bitmap(640, 480);
        private byte[] colorPixels;

        private Queue<DepthColor[,]> depthQueue = new Queue<DepthColor[,]>(), depthPool = new Queue<DepthColor[,]>();

        private DepthImagePixel[] depthPixels;
        private DepthImagePoint[] depthPoints;
        private ColorImagePoint[] colorPoints;

        private bool readyForWrite = true;
        private Thread bitmapFiller, depthUpdater;

        System.Windows.Threading.DispatcherOperation viewer;


        PCViewer g;



        public MainWindow()
        {
            InitializeComponent();

        }

        private void FillBitmap()
        {
            if (readyForWrite)
            {
                System.Drawing.Color c;
                for (int i = 0; i < this.depthPoints.Length; i++)
                {
                    c = System.Drawing.Color.FromArgb(150, (byte)this.depthPoints[i].Depth, 0, 0);
                    this.image.SetPixel(i % 640, i / 640, c);
                }
            }
        }

        private void WindowLoaded(object sender, RoutedEventArgs e)
        {


            foreach (var potentialKinect in KinectSensor.KinectSensors)
            {
                if (potentialKinect.Status == KinectStatus.Connected)
                {
                    this.kinect = potentialKinect;
                    break;
                }
            }

            if (this.kinect != null)
            {


                this.kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                this.kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                this.colorPixels = new byte[this.kinect.ColorStream.FramePixelDataLength];
                this.depthPixels = new DepthImagePixel[this.kinect.DepthStream.FramePixelDataLength];

                this.imageBitmap = new WriteableBitmap(this.kinect.ColorStream.FrameWidth, this.kinect.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.depthBitmap = new WriteableBitmap(this.kinect.DepthStream.FrameWidth, this.kinect.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                this.depthPoints = new DepthImagePoint[640 * 480];
                this.colorPoints = new ColorImagePoint[640 * 480];
                bitmapFiller = new Thread(new ThreadStart(FillBitmap));
                depthUpdater = new Thread(new ThreadStart(UpdateDepthData));
                // this.cdMap = new DepthColorPixel[640, 480];

                this.Image.Source = this.imageBitmap;
                this.Depth.Source = this.depthBitmap;

                this.KeyDown += new KeyEventHandler(OnButtonKeyDown);

                //this.kinect.ColorFrameReady += this.KinectColorFrameReady;
                //this.kinect.DepthFrameReady += this.KinectDepthFrameReady;
                this.kinect.AllFramesReady += this.KinectAllFramesReady;
                try
                {
                    this.kinect.Start();
                }
                catch (IOException)
                {
                    this.kinect = null;
                    this.Title = "NOT READY";
                }

                this.kinect.ElevationAngle = 10;

                bitmapFiller.Start();
                depthUpdater.Start();

                for (int i = 0; i < 10; i++)
                {
                    depthPool.Enqueue(new DepthColor[640, 480]);
                }

                g = new PCViewer(depthQueue, depthPool);
                viewer = Dispatcher.BeginInvoke(new Action(() =>
                {
                    g.Run();
                }));

            }


        }

        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.bitmapFiller.Abort();
            if (null != this.kinect)
            {
                this.kinect.Stop();
            }
            int x = this.image.Height;
            // Bitmap temp = new Bitmap(this.image);
            //temp.Save("file.png");
            g.Exit();
        }

        private void UpdateDepthData()
        {
            int baseindex;
            if (this.depthPool.Count > 0)
            {
                DepthColor[,] depth;
                lock (this.depthPool)
                {
                    depth = this.depthPool.Dequeue();
                }

                for (int i = 0; i < 640 * 480; i++)
                {
                    int x = i % 640;
                    int y = i / 640;
                    if ((this.colorPoints[i].X >= 0 && this.colorPoints[i].X < 640 && this.colorPoints[i].Y >= 0 && this.colorPoints[i].Y < 480))
                    {

                        depth[x, y].Depth = this.depthPixels[i].Depth;

                        ColorImagePoint p = this.colorPoints[i];
                        baseindex = (p.X + p.Y * 640) * 4;

                        depth[x, y].Color.X = this.colorPixels[baseindex + 2]/255f;
                        depth[x, y].Color.Y = this.colorPixels[baseindex + 1] / 255f;
                        depth[x, y].Color.Z = this.colorPixels[baseindex] / 255f;


                        //depth[x, y].Color = System.Drawing.Color.FromArgb(this.colorPixels[i * 4], this.colorPixels[i * 4 + 1], this.colorPixels[i * 4 + 2]);
                        //depth[x, y].Depth = this.depthPixels[this.colorPoints[i].X + this.colorPoints[i].Y * 640].Depth;
                        depth[x, y].UpToDate = true;
                    }

                }

                /*foreach (DepthImagePoint p in depthPoints)
                {
                    if (p.X >= 0 && p.X < 640 && p.Y >= 0 && p.Y < 480)
                    {
                        depth[p.X, p.Y].Depth = p.Depth;
                        depth[p.X, p.Y].Color = System.Drawing.Color.FromArgb(255, 0, 250);
                        depth[p.X, p.Y].UpToDate = true;
                    }
                }
                */

                lock (this.depthQueue)
                {
                    this.depthQueue.Enqueue(depth);
                }
            }
            else
            {
                //Thread.Sleep(100);
            }
        }

        private void OnButtonKeyDown(object sender, KeyEventArgs e)
        {

            System.Diagnostics.Debug.WriteLine(e.Key.ToString());
        }

        private void KinectColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            this.Title = "Working...";
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyPixelDataTo(this.colorPixels);
                    this.imageBitmap.WritePixels(
                        new Int32Rect(0, 0, this.imageBitmap.PixelWidth, this.imageBitmap.PixelHeight),
                        this.colorPixels,
                        this.imageBitmap.PixelWidth * sizeof(int),
                        0);

                    //Creates Bitmap
                    /*using (MemoryStream stream = new MemoryStream())
                    {
                        BitmapEncoder encoder = new BmpBitmapEncoder();
                        encoder.Frames.Add(BitmapFrame.Create((BitmapSource)imageBitmap));
                        encoder.Save(stream);
                        this.image.Dispose();
                        this.image = new System.Drawing.Bitmap(stream);
                    }*/



                }

            }
        }


        private void KinectDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    this.depthBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                        this.depthPixels,
                        this.imageBitmap.PixelWidth * sizeof(int),
                        0);

                }
                this.kinect.CoordinateMapper.MapColorFrameToDepthFrame(ColorImageFormat.RgbResolution640x480Fps30, DepthImageFormat.Resolution640x480Fps30, this.depthPixels, this.depthPoints);

                depthUpdater = new Thread(new ThreadStart(UpdateDepthData));
                depthUpdater.Start();
                //this.UpdateDepthData();
                //this.fillBitmap();
            }
        }

        private void KinectAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyPixelDataTo(this.colorPixels);
                    this.imageBitmap.WritePixels(
                        new Int32Rect(0, 0, this.imageBitmap.PixelWidth, this.imageBitmap.PixelHeight),
                        this.colorPixels,
                        this.imageBitmap.PixelWidth * sizeof(int),
                        0);
                }

            }
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    this.depthBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                        this.depthPixels,
                        this.imageBitmap.PixelWidth * sizeof(int),
                        0);

                }
                //this.kinect.CoordinateMapper.MapColorFrameToDepthFrame(ColorImageFormat.RgbResolution640x480Fps30, DepthImageFormat.Resolution640x480Fps30, this.depthPixels, this.depthPoints);
                this.kinect.CoordinateMapper.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, this.depthPixels, ColorImageFormat.RgbResolution640x480Fps30, this.colorPoints);

                depthUpdater = new Thread(new ThreadStart(UpdateDepthData));
                depthUpdater.Start();

            }

        }


    }
}
