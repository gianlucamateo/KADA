using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using Color = System.Drawing.Color;
using Microsoft.Xna.Framework;

namespace KADA
{
    class RGBHistogram : IHistogram
    {
        public float[, ,] colorBins;
        private readonly int RESOLUTION;
        private readonly int BINSIZE;
        public XYZFileLoader.BrickColor color;
        Matrix RGBToYUV, YUVToRGB;
        PipelineDataContainer dataContainer;
        private Vector3 YUV;

        private float threshold;
        public RGBHistogram(Bitmap image, float threshold, int resolution, XYZFileLoader.BrickColor color, PipelineDataContainer dataContainer, Vector3 YUV)
        {
            this.dataContainer = dataContainer;
            this.RGBToYUV = new Matrix(0.299f, 0.587f, 0.144f, 0f, -0.14713f, -0.28886f, 0.436f, 0f, 0.615f, -0.51499f, -0.10001f, 0f, 0f, 0f, 0f, 1f);
            this.RGBToYUV = Matrix.Transpose(this.RGBToYUV);
            this.YUVToRGB = Matrix.Invert(RGBToYUV);
            this.RESOLUTION = resolution;
            BINSIZE = 256 / RESOLUTION;
            colorBins = new float[RESOLUTION, RESOLUTION, RESOLUTION];
            this.threshold = threshold;
            this.YUV = YUV;
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

                    if (dataContainer.UseYUV)
                    {
                        Vector3 color = new Vector3(c.R, c.G, c.B);
                        color = Vector3.Transform(color, RGBToYUV);
                        color /= 255;
                        color.X = 0.5f;
                        color *= 255;
                        color = Vector3.Transform(color, YUVToRGB);
                        int r = Math.Max(Math.Min((int)color.X, 255), 0);
                        int g = Math.Max(Math.Min((int)color.Y, 255), 0);
                        int b = Math.Max(Math.Min((int)color.Z, 255), 0);

                        c = Color.FromArgb(r, g, b);
                    }

                    if (!c.IsEmpty)
                    {
                        int r = c.R / BINSIZE;
                        int g = c.G / BINSIZE;
                        int b = c.B / BINSIZE;
                        colorBins[r, g, b]++;
                        //System.Diagnostics.Debug.WriteLine(colorBins[r, g, b] + " " + r + " " + g + " " + b);
                        // System.Diagnostics.Debug.WriteLine(c.R + " " + c.G + " " + c.B);
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

        public int GetValue(Vector3 color)
        {
            if (!dataContainer.UseYUV)
            {
                if (color.Length() < 50)
                    return 0;
                if (color.Length() > 400)
                    return 0;
            }


            /*float green = g[(int)color.Y];
            float red = r[(int)color.X];
            float blue = r[(int)color.Z];*/
            if (dataContainer.UseYUV)
            {
                //color = color + new Vector3(123, 123, 123);
                //color = Vector3.Transform(color, YUVToRGB);
                Vector3 distance = color - this.YUV;
                if (distance.Length() < 40)
                {
                    return 1;
                }
                return 0;
            }

            int r = Math.Max(Math.Min((int)color.X / BINSIZE, RESOLUTION - 1), 0);
            int g = Math.Max(Math.Min((int)color.Y / BINSIZE, RESOLUTION - 1), 0);
            int b = Math.Max(Math.Min((int)color.Z / BINSIZE, RESOLUTION - 1), 0);

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

        public XYZFileLoader.BrickColor GetColor()
        {
            return this.color;
        }
    }
}
