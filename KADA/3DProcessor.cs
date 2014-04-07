using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;
using System.Threading;
using System.Collections.Concurrent;
using Microsoft.Xna.Framework;

namespace KADA
{
    class _3DProcessor
    {
        private ConcurrentQueue<DepthColor[,]> renderQueue, processingQueue;
        private ConcurrentQueue<Vector3> centers;
        private readonly float THRESHOLD = 100;

        public _3DProcessor(ConcurrentQueue<DepthColor[,]> processingQueue, ConcurrentQueue<DepthColor[,]> renderQueue, ConcurrentQueue<Vector3> centers)
        {
            this.renderQueue = renderQueue;
            this.processingQueue = processingQueue;
            this.centers = centers;
        }

        public void generateCenter(Object dcIn)
        {
            DepthColor[,] dc;
            if (processingQueue.TryDequeue(out dc) == false)
                return;

            float x = 0, y = 0, z = 0;
            int counter = 0;
            DepthColor c;
            for (int xP = 0; xP < dc.GetLength(0); xP++)
            {
                for (int yP = 0; yP < dc.GetLength(1); yP++)
                {
                    c = dc[xP, yP];
                    if (c.Position.Z != 0)
                    {
                        x += c.Position.X;
                        y += c.Position.Y;
                        z += c.Position.Z;
                        counter++;
                    }
                }
            }
            x /= counter;
            y /= counter;
            z /= counter;
            Vector3 center = new Vector3(x, y, z);

            x = 0;
            y = 0;
            z = 0;
            counter = 0;
            for (int xP = 0; xP < dc.GetLength(0); xP++)
            {
                for (int yP = 0; yP < dc.GetLength(1); yP++)
                {
                    c = dc[xP, yP];
                    if (c.Position.Z != 0)
                    {
                        float dist;
                        Vector3.Distance(ref center, ref c.Position, out dist);
                        if (dist < THRESHOLD)
                        {
                            x += c.Position.X;
                            y += c.Position.Y;
                            z += c.Position.Z;
                            counter++;
                        }
                    }
                }
            }
            x /= counter;
            y /= counter;
            z /= counter;
            center = new Vector3(x, y, z);
            this.centers.Enqueue(center);
            this.renderQueue.Enqueue(dc);
        }
    }
}
