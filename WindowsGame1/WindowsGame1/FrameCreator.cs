using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.Kinect;
using System.Threading;

namespace KADA
{
    public class FrameCreator
    {
        private KinectSensor kinect;
        private PipelineDataContainer dataContainer;
        private PipelineManager manager;
        private int number;
        private StreamWriter file;

        public FrameCreator(PipelineDataContainer dataContainer, PipelineManager manager)
        {
            this.file = new StreamWriter("timings.csv");
            this.manager = manager;
            this.dataContainer = dataContainer;
            Thread frameCreator = new Thread(new ThreadStart(()=>kinectFramePull()));
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
                this.dataContainer.COLORLENGTH = this.kinect.ColorStream.FramePixelDataLength;
                this.dataContainer.DEPTHLENGTH = this.kinect.DepthStream.FramePixelDataLength;


                try
                {
                    this.kinect.Start();
                }
                catch (IOException)
                {
                    this.kinect = null;
                }

               // this.kinect.AllFramesReady += this.KinectAllFramesReady;
                frameCreator.Start();
                this.number = 0;
            }
        }

        //Stage 0
        PipelineContainer container;
        private void kinectFramePull()
        {
            while (this.dataContainer.run)
            {
                if (this.manager.recycle.Count > 3)
                {
                    while (this.manager.recycle.TryDequeue(out container) == false)
                    {
                        Thread.Sleep(dataContainer.SLEEPTIME);
                    }
                }
                else
                {
                    Thread.Sleep(dataContainer.SLEEPTIME);
                    continue;
                }

                using (ColorImageFrame colorFrame = kinect.ColorStream.OpenNextFrame(0))
                {
                    if (colorFrame == null)
                    {
                        manager.recycle.Enqueue(container);
                        continue;
                    }
                    colorFrame.CopyPixelDataTo(container.colorPixels);                    
                }
                using (DepthImageFrame depthFrame = kinect.DepthStream.OpenNextFrame(50))
                {
                    if (depthFrame == null)
                    {
                        manager.recycle.Enqueue(container);
                        continue;
                    }
                    depthFrame.CopyDepthImagePixelDataTo(container.depthPixels);
                }
                int stage = 0;
                TimeSpan current;
                DateTime previous = DateTime.MinValue;
                if (dataContainer.deNoiseAndICP)
                {
                    this.file.Write(container.number + " ");
                    foreach (DateTime t in container.timings)
                    {
                        current = t.Subtract(previous);
                        this.file.Write("; Stage: " + stage++ + "; : " + current);
                        previous = t;
                    }

                    this.file.WriteLine("");
                }

                container.timings = new List<DateTime>();
                Random r = new Random();
                container.number = ++this.number;
                container.stage = 0;
                container.qi.Clear();

                
                

                this.kinect.CoordinateMapper.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, container.depthPixels, ColorImageFormat.RgbResolution640x480Fps30, container.colorPoints);

                dataContainer.recordGenerationTick();
                container.timings.Add(DateTime.Now);
                manager.processingQueues[++container.stage].Enqueue(container);
                
            }
        }


        //Stage 0
        private void KinectAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            
            if (this.manager.recycle.Count > 3)
            {
                while (this.manager.recycle.TryDequeue(out container) == false)
                {
                    Thread.Sleep(dataContainer.SLEEPTIME);
                }
            }


            if (container == null || dataContainer.COLORLENGTH == 0 || dataContainer.DEPTHLENGTH == 0)
            {
                return;
            }

            int stage = 0;
            TimeSpan current;
            DateTime previous = DateTime.MinValue;
            if (dataContainer.deNoiseAndICP)
            {
                this.file.Write(container.number + ": ");
                foreach (DateTime t in container.timings)
                {
                    current = t.Subtract(previous);
                    this.file.Write(" Stage: " + stage++ + " : " + current);
                    previous = t;
                }

                this.file.WriteLine("");
            }

            container.timings = new List<DateTime>();
            Random r = new Random();
            container.number = ++this.number;
            container.stage = 0;
            container.qi.Clear();

            /* for (int x = 0; x < 640*480; x++)
             {
                
                     container.colorPoints[x] = new ColorImagePoint();
                
             }*/

            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {

                    colorFrame.CopyPixelDataTo(container.colorPixels);
                    /*this.imageBitmap.WritePixels(
                        new Int32Rect(0, 0, this.imageBitmap.PixelWidth, this.imageBitmap.PixelHeight),
                        this.colorPixels,
                        this.imageBitmap.PixelWidth * sizeof(int),
                        0);*/
                }

            }
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {

                    depthFrame.CopyDepthImagePixelDataTo(container.depthPixels);

                    /*this.depthBitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                        this.depthPixels,
                        this.imageBitmap.PixelWidth * sizeof(int),
                        0);*/

                }
                //this.kinect.CoordinateMapper.MapColorFrameToDepthFrame(ColorImageFormat.RgbResolution640x480Fps30, DepthImageFormat.Resolution640x480Fps30, this.depthPixels, this.depthPoints);
                this.kinect.CoordinateMapper.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, container.depthPixels, ColorImageFormat.RgbResolution640x480Fps30, container.colorPoints);

                /*Boolean processColors = _3Dprocessor.g.saveColors;
                System.Threading.ThreadPool.QueueUserWorkItem(new System.Threading.WaitCallback(UpdateDepthData), processColors);
                depthUpdater = new Thread(new ThreadStart(() => UpdateDepthData(saveToFile)));
                depthUpdater.Start();*/
                /*
                if (!this._2Dprocessor.BackgroundReady() && _3Dprocessor.g.generateBackground)
                {
                    imageProcessorThread = new Thread(new ThreadStart(() => _2Dprocessor.GenerateBackground(depthPixels)));
                    imageProcessorThread.Start();
                    // this.processor.GenerateBackground(this.depthPixels);
                }*/
            }
            dataContainer.recordGenerationTick();
            // manager.enqueue(container);
            container.timings.Add(DateTime.Now);
            manager.processingQueues[++container.stage].Enqueue(container);
        }
    }
}
