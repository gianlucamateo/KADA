using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.Kinect;
using System.Threading;
using Microsoft.Xna.Framework;

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

                dataContainer.g = Microsoft.Xna.Framework.Vector3.Zero;
                for (int i = 0; i < 40; i++)
                {
                    dataContainer.g -= new Microsoft.Xna.Framework.Vector3(-this.Kinect.AccelerometerGetCurrentReading().X, this.Kinect.AccelerometerGetCurrentReading().Y, -this.Kinect.AccelerometerGetCurrentReading().Z);
                    Thread.Sleep(50);
                }
                dataContainer.g.Normalize();

                Vector3 axis = Vector3.Cross(Vector3.Up, dataContainer.g);
                axis.Normalize();

                float angle = (float)Math.Acos(Vector3.Dot(Vector3.Up, dataContainer.g) / (Vector3.Up.Length() * dataContainer.g.Length()));

                Matrix initialRot = Matrix.CreateFromAxisAngle(axis, angle);
                dataContainer.BaseRotation = initialRot;
                dataContainer.lastConfidentR = initialRot;

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
                if (this.Manager.Recycle.Count > 1)
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
            if (container.Number % 30 == 0)
            {
                GC.Collect();
            }
            DataContainer.recordGenerationTick();
            container.Timings.Add(DateTime.Now);
            Manager.ProcessingQueues[++container.Stage].Enqueue(container);
        }

        private void WriteDiagnostics()
        {
            int stage = 0;
            TimeSpan current;
            
            if (DataContainer.DeNoiseAndICP)
            {
                /*for(int i = 0; i<8;i++){
                    this.File2.Write(Manager.ProcessingQueues[i].Count + ";");
                }*/
                this.File2.WriteLine("");
                this.File.Write("Bricks: ;" + DataContainer.model.Bricks.Count + ";");             
                for (int i = 2; i < container.Timings.Count; i+=2)
                {
                    DateTime t = container.Timings[i];
                    current = t - container.Timings[i-1];
                    this.File.Write(current.Milliseconds + ";");                                      
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
