using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.Kinect;

namespace KADA
{
    public class FrameCreator
    {
        private KinectSensor kinect;
        private PipelineDataContainer dataContainer;
        private PipelineManager manager;
        
        public FrameCreator(PipelineDataContainer dataContainer, PipelineManager manager)
        {
            this.manager = manager;
            this.dataContainer = dataContainer;
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
                
                this.kinect.AllFramesReady += this.KinectAllFramesReady;


            }
        }
        //Stage 0
        PipelineContainer container;
        private void KinectAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            
            this.manager.recycle.TryDequeue(out container);
            
            if (container == null||dataContainer.COLORLENGTH == 0 || dataContainer.DEPTHLENGTH == 0)
            {
                return;
            }
            container.stage = 0;

            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    container.colorPixels = new byte[dataContainer.COLORLENGTH];
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
                    container.depthPixels = new DepthImagePixel[this.dataContainer.DEPTHLENGTH];
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
            manager.processingQueues[++container.stage].Enqueue(container);
        }
    }
}
