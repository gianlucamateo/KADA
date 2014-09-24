using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using Color = System.Drawing.Color;
using Microsoft.Xna.Framework;

namespace KADA
{
    class Histogram
    {
        public float[, ,] colorBins;
        private readonly int RESOLUTION;
        private readonly int BINSIZE;
        public XYZFileLoader.BrickColor color;

        private float threshold;
        public Histogram(Bitmap image, float threshold, int resolution, XYZFileLoader.BrickColor color)
        {
            this.RESOLUTION = resolution;
            BINSIZE = 256 / RESOLUTION;
            colorBins = new float[RESOLUTION,RESOLUTION,RESOLUTION];
            this.threshold = threshold;
            this.generate(image);
            this.color = color;
        }

        public void generate(Bitmap image)
        {
            for (int x = 0; x < image.Width; x++)
            {
                for (int y = 0; y < image.Height; y++)
                {
                    Color c = image.GetPixel(x, y);

                    
                    if (!c.IsEmpty)
                    {
                        int r = c.R / BINSIZE;
                        int g = c.G / BINSIZE;
                        int b = c.B / BINSIZE;
                        colorBins[r, g, b]++;
                    }

                }
            }
            this.colorBins[RESOLUTION - 1, RESOLUTION - 1, RESOLUTION - 1] = 0;
            this.colorBins[0, 0, 0] = 0;
            /*float green = 0, red = 0, blue = 0;
            for (int i = 0; i < 255; i++)
            {
                g[i] = (float)Math.Log10(g[i]);
                r[i] = (float)Math.Log10(r[i]);
                b[i] = (float)Math.Log10(b[i]);
            }

            for (int i = 0; i < 255; i++)
            {
                if (g[i] > 0)
                    green += g[i];
                if (r[i] > 0)
                    red += r[i];
                if (b[i] > 0)
                    blue += b[i];
            }*/

            /*for (int i = 0; i < 255; i++)
            {
                g[i] /= green;
                r[i] /= red;
                b[i] /= blue;
            }*/
        }

        public int getValue(Vector3 color)
        {
            if (color.Length() < 50)
                return 0;
            if (color.Length() > 400)
                return 0;


            /*float green = g[(int)color.Y];
            float red = r[(int)color.X];
            float blue = r[(int)color.Z];*/
            int r = (int)color.X / BINSIZE;
            int g = (int)color.Y / BINSIZE;
            int b = (int)color.Z / BINSIZE;

            //bool isbrick = green > threshold && red > threshold && blue > threshold;
            bool isbrick = colorBins[r, g, b] > threshold;

            if (isbrick)
                return 1;
            /*sum += r[(int)color.X];
            sum += g[(int)color.Y];
            sum += b[(int)color.Z];
            if(sum>0.01f)
                return 1;*/
            return 0;

        }
    }
}
