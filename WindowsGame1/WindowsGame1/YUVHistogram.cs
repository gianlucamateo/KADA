using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using System.Drawing;
namespace KADA
{
    class YUVHistogram : IHistogram
    {
        int counter = 0;
        Bitmap b;
        Matrix RGBToYUV, YUVToRGB;
        KADA.BrickColor brickColor;
        private int[,] values;

        public YUVHistogram(Bitmap mask, Bitmap blacklist, KADA.BrickColor brickColor)
        {
            this.brickColor = brickColor;
            this.values = new int[512, 512];
            System.Drawing.Color c;
            for (int x = 0; x < mask.Width; x++)
            {
                for (int y = 0; y < mask.Height; y++)
                {
                    c = mask.GetPixel(x, y);
                    float brightness = c.GetBrightness();
                    if (brightness > 0.5f)
                    {
                        values[x, y] = 0;
                    }
                    else
                    {
                        values[x, y] = 1;//(int)brickColor;
                    }
                    c = blacklist.GetPixel(x, y);
                    brightness = c.GetBrightness();
                    if (brightness < 0.5f)
                    {
                        values[x , y] = 0;
                    }

                }
            }

            b = new Bitmap(512, 512);

            this.RGBToYUV = new Matrix(0.299f, 0.587f, 0.144f, 0f, -0.14713f, -0.28886f, 0.436f, 0f, 0.615f, -0.51499f, -0.10001f, 0f, 0f, 0f, 0f, 1f);
            this.RGBToYUV = Matrix.Transpose(this.RGBToYUV);
            this.YUVToRGB = Matrix.Invert(RGBToYUV);
        }
        public int GetValue(Vector3 color)
        {
            /*Vector3 writeColor = Vector3.Transform(color, YUVToRGB);
            writeColor b= Vector3.Min(writeColor, new Vector3(255, 255, 255));
            writeColor = Vector3.Max(writeColor, new Vector3(0, 0, 0));
            b.SetPixel((int)color.Y + 255, (int)color.Z + 255, System.Drawing.Color.FromArgb((int)writeColor.X, (int)writeColor.Y, (int)writeColor.Z));
            counter++;

            if (counter == 4000)
            {
                b.Save("colordist.bmp");
                System.Diagnostics.Debug.WriteLine("saved");
                counter++;
            }
            else if(counter<4000)
            {
                System.Diagnostics.Debug.WriteLine(counter);
            }*/

            int x = ((int)color.Y + 255);
            int y = ((int)color.Z + 255);
            //System.Diagnostics.Debug.WriteLine(x+" "+y);
            return this.values[x, y];
        }


        public KADA.BrickColor GetColor()
        {
            return this.brickColor;
        }
    }
}
