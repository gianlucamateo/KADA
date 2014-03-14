using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.Concurrent;
using Microsoft.Kinect;

namespace KADA
{
    class ImageProcessor
    {
        private ConcurrentQueue<DepthColor[,]> renderQueue;
        private DepthImagePixel[] background;
        private bool backgroundReady=false;
        private int backgroundFrameCount=0;
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
                        localDepth = (short)(localDepth*0.989f);
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

    }
}
