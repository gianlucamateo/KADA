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
using Microsoft.Xna.Framework;
using System.Collections.Concurrent;

using Matrix = Microsoft.Xna.Framework.Matrix;
using System.Diagnostics;




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

        private ConcurrentQueue<DepthColor[,]> renderQueue = new ConcurrentQueue<DepthColor[,]>(),
            depthPool = new ConcurrentQueue<DepthColor[,]>(), processingQueue = new ConcurrentQueue<DepthColor[,]>();

        private ConcurrentQueue<Vector3> centers = new ConcurrentQueue<Vector3>();

        private ConcurrentQueue<Matrix> rotations = new ConcurrentQueue<Matrix>();

        private DepthImagePixel[] depthPixels;
        private DepthImagePoint[] depthPoints;
        private ColorImagePoint[] colorPoints;

        private bool readyForWrite = true;
        private Thread bitmapFiller, depthUpdater, imageProcessorThread;


        System.Windows.Threading.DispatcherOperation pcviewer;


        PCViewer g;

        ImageProcessor imageProcessor;

        _3DProcessor _3Dprocessor;



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
                depthUpdater = new Thread(new ThreadStart(() => UpdateDepthData(false)));
                imageProcessorThread = new Thread(new ThreadStart(() => imageProcessor.GenerateBackground(depthPixels)));
                // this.cdMap = new DepthColorPixel[640, 480];

                this.Image.Source = this.imageBitmap;
                this.Depth.Source = this.depthBitmap;


                this.KeyDown += new KeyEventHandler(OnButtonKeyDown);

                //this.kinect.ColorFrameReady += this.KinectColorFrameReady;
                //this.kinect.DepthFrameReady += this.KinectDepthFrameReady;


                //this.kinect.ElevationAngle = 10;
                //bitmapFiller.Start();
                //depthUpdater.Start();

                for (int i = 0; i < 6; i++)
                {
                    depthPool.Enqueue(new DepthColor[640, 480]);
                }

                imageProcessor = new ImageProcessor(processingQueue);



                g = new PCViewer(renderQueue, depthPool);//, this);

                pcviewer = Dispatcher.BeginInvoke(new Action(() =>
                {
                    g.Run();
                }));

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

                _3Dprocessor = new _3DProcessor(processingQueue, renderQueue, centers, rotations, g.offset);

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




        private void UpdateDepthData(Object saveToFileOb)
        {
            int baseindex;

            bool processColors = (Boolean)saveToFileOb;
            DepthColor[,] depth = null;
            DepthImagePixel[] dPixels = this.depthPixels;
            DepthImagePixel[] background = null;

            if (!this.depthPool.TryDequeue(out depth))
                return;
            if (this.imageProcessor.BackgroundReady())
            {
                background = imageProcessor.getBackground();
            }
            for (int i = 0; i < 640 * 480; i++)
            {
                int x = i % 640;
                int y = 480 - (i / 640) - 1;
                if ((this.colorPoints[i].X >= 0 && this.colorPoints[i].X < 640 && this.colorPoints[i].Y >= 0 && this.colorPoints[i].Y < 480))
                {

                    depth[x, y].Depth = dPixels[i].Depth;
                    float scale = -dPixels[i].Depth / 670.63f;
                    depth[x, y].Position.X = (x - 320) * scale;
                    depth[x, y].Position.Y = -(y-240) * scale;
                    depth[x, y].Position.Z = -dPixels[i].Depth;
                    if (background != null)
                    {
                        bool drop = dPixels[i].Depth > background[i].Depth;
                        if (drop)
                        {
                            depth[x, y].Position *= 0;
                        }
                    }

                    ColorImagePoint p = this.colorPoints[i];
                    baseindex = (p.X + p.Y * 640) * 4;
                    Vector3 c = depth[x, y].Color;

                    c.X = this.colorPixels[baseindex + 2];
                    c.Y = this.colorPixels[baseindex + 1];
                    c.Z = this.colorPixels[baseindex];

                    depth[x, y].Color = c / 255f;

                    depth[x, y].UpToDate = true;
                }

            }
            if (processColors)
            {
                System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(this.imageProcessor.eliminateColor), depth);
                ImageProcessor.resetEvent.WaitOne();
                System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(this._3Dprocessor.generateCenter));
                Vector3 center;
                if (centers.TryDequeue(out center) == true)
                {
                    this.g.SetBrickTranslate(Matrix.CreateTranslation(center));
                    Matrix R;
                    if(rotations.TryDequeue(out R)){
                        this.g.SetBrickRotation(R);
                    }
                    //Debug.WriteLine(center);
                }
                //this.renderQueue.Enqueue(depth);
                
            }

           // if (this.processor.ColorReady())
            //{

           //     System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(processor.eliminateColor), depth);
          //      ImageProcessor.resetEvent.WaitOne();
           //     //System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(processor.scanNormals), depth);
          //          //imageProcessor = new Thread(new ThreadStart(() => processor.eliminateColor(depth)));
          //          //imageProcessor.Start();
                
               
                    /*this.depthPool.Enqueue(depth);
                    return;*/
                
           // }
            else
            {
                this.renderQueue.Enqueue(depth);
            }
        }

        private void OnButtonKeyDown(object sender, KeyEventArgs e)
        {
            _3Dprocessor.scanNormals();

            System.Diagnostics.Debug.WriteLine(e.Key.ToString());
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

                Boolean processColors = g.saveColors;
                System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(UpdateDepthData), processColors);
                /*depthUpdater = new Thread(new ThreadStart(() => UpdateDepthData(saveToFile)));
                depthUpdater.Start();*/

                if (!this.imageProcessor.BackgroundReady() && this.g.generateBackground)
                {
                    imageProcessorThread = new Thread(new ThreadStart(() => imageProcessor.GenerateBackground(depthPixels)));
                    imageProcessorThread.Start();
                    // this.processor.GenerateBackground(this.depthPixels);
                }



            }


        }


    }
}
