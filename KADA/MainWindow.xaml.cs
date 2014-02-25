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

using System.IO;
using Microsoft.Kinect;

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

        private byte[] colorPixels;
       
        private DepthImagePixel[] depthPixels;

        public MainWindow()
        {
            InitializeComponent();
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

                this.Image.Source = this.imageBitmap;
                this.Depth.Source = this.depthBitmap;

                this.kinect.ColorFrameReady += this.KinectColorFrameReady;
                this.kinect.DepthFrameReady += this.KinectDepthFrameReady;
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
                
            }


        }

        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.kinect)
            {
                this.kinect.Stop();
            }
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
            }
        }

    }
}
