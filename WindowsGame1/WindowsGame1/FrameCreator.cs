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
        private KinectSensor Kinect;
        private PipelineDataContainer DataContainer;
        private PipelineManager Manager;
        private int Number;
        private StreamWriter File, File2;
        private Queue<DepthImagePixel[]> DepthQueue = new Queue<DepthImagePixel[]>();

        public FrameCreator(PipelineDataContainer dataContainer, PipelineManager manager)
        {
            this.File = new StreamWriter("timings.csv");
            this.File2 = new StreamWriter("loads.csv");
            this.Manager = manager;
            this.DataContainer = dataContainer;
            Thread frameCreator = new Thread(new ThreadStart(() => KinectFramePull()));
            foreach (var potentialKinect in KinectSensor.KinectSensors)
            {
                if (potentialKinect.Status == KinectStatus.Connected)
                {
                    this.Kinect = potentialKinect;
                    break;
                }
            }
            for (int i = 0; i < 3; i++)
            {
                this.DepthQueue.Enqueue(new DepthImagePixel[640 * 480]);
            }
            if (this.Kinect != null)
            {

                this.Kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                this.Kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
                this.DataContainer.COLORLENGTH = this.Kinect.ColorStream.FramePixelDataLength;
                this.DataContainer.DEPTHLENGTH = this.Kinect.DepthStream.FramePixelDataLength;


                try
                {
                    this.Kinect.Start();
                }
                catch (IOException)
                {
                    this.Kinect = null;
                }

                // this.kinect.AllFramesReady += this.KinectAllFramesReady;
                frameCreator.Start();
                this.Number = 0;
            }
        }

        // Stage 0
        PipelineContainer container;
        private void KinectFramePull()
        {
            while (this.DataContainer.Run)
            {
                if (this.Manager.Recycle.Count > 3 && this.DataContainer.backgroundEvaluator.ModificationRunning==false)
                {
                    while (this.Manager.Recycle.TryDequeue(out container) == false)
                    {
                        Thread.Sleep(DataContainer.SLEEPTIME);
                    }
                }
                else
                {
                    Thread.Sleep(DataContainer.SLEEPTIME);
                    continue;
                }
                //container.Timings.Add(DateTime.Now);



                using (ColorImageFrame colorFrame = Kinect.ColorStream.OpenNextFrame(0))
                {
                    if (colorFrame == null)
                    {
                        Manager.Recycle.Enqueue(container);
                        continue;
                    }
                    if (DepthQueue.Count > 0)
                    {
                        //DepthImagePixel[] arr = depthqueue.Dequeue();
                        colorFrame.CopyPixelDataTo(container.ColorPixels);
                        //container.depthPixels = (DepthImagePixel[])arr.Clone();
                        //depthqueue.Enqueue(arr);
                    }
                }

                using (DepthImageFrame depthFrame = Kinect.DepthStream.OpenNextFrame(50))
                {
                    if (depthFrame == null)
                    {
                        Manager.Recycle.Enqueue(container);
                        continue;
                    }

                    //DepthImagePixel[] arr = depthqueue.Dequeue();
                    //depthFrame.CopyDepthImagePixelDataTo(arr);
                    //depthqueue.Enqueue(arr);
                    depthFrame.CopyDepthImagePixelDataTo(container.DepthPixels);
                }


                WriteDiagnostics();

                this.Kinect.CoordinateMapper.MapDepthFrameToColorFrame(DepthImageFormat.Resolution640x480Fps30, container.DepthPixels, ColorImageFormat.RgbResolution640x480Fps30, container.ColorPoints);

                AddToQueue();

            }
        }

        private void AddToQueue()
        {
            DataContainer.recordGenerationTick();
            //container.Timings.Add(DateTime.Now);
            Manager.ProcessingQueues[++container.Stage].Enqueue(container);
        }

        private void WriteDiagnostics()
        {
            int stage = 0;
            TimeSpan current;
            DateTime previous = DateTime.MinValue;
            if (DataContainer.DeNoiseAndICP)
            {
                for(int i = 0; i<8;i++){
                    this.File2.Write(Manager.ProcessingQueues[i].Count + ";");
                }
                this.File2.WriteLine("");
                this.File.Write(container.Number + " ");
                for (int i = 0; i < container.Timings.Count - 1; i+=2)
                {
                    DateTime t = container.Timings[i];
                    current = t.Subtract(previous);
                    this.File.Write("; Stage: " + stage++ + ";  " + current);
                    previous = t;
                    t = container.Timings[i+1];
                    current = t.Subtract(previous);
                    previous = t;
                    this.File.Write("; "+ current);
                    
                }
                if (container.Timings.Count > 1)
                {
                    DateTime end = container.Timings[container.Timings.Count - 1];
                    DateTime beginning = container.Timings[0];
                    current = end.Subtract(beginning);
                    this.File.Write(";Total ; " + current);
                }

                this.File.WriteLine("");
            }

            container.Timings = new List<DateTime>();

            container.Number = ++this.Number;
            container.Stage = 0;
            container.Qi.Clear();
        }
    }
}
