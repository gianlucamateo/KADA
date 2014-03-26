using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Threading;
using System.Collections;
using Microsoft.Xna.Framework;
using Color = System.Drawing.Color;

namespace KADA
{
    class HistogramReader
    {
        static int Main(String[] args)
        {
            List<Histogram> histograms = new List<Histogram>();

            Bitmap red = (Bitmap)Image.FromFile("../../Red_cleaned_filled.png", true);
            Histogram r = new Histogram(red, 150, 8);
            histograms.Add(r);

            Bitmap green = (Bitmap)Image.FromFile("../../Green_cleaned_filled.png", true);
            Histogram g = new Histogram(green, 25, 16);
            histograms.Add(g);

            Bitmap blue = (Bitmap)Image.FromFile("../../Blue_cleaned_filled.png", true);
            Histogram b = new Histogram(blue,100, 8);
            histograms.Add(b);

            Bitmap yellow = (Bitmap)Image.FromFile("../../Yellow_cleaned_filled.png", true);
            Histogram ye = new Histogram(yellow, 150, 8);
            histograms.Add(ye);


            Bitmap test = (Bitmap)Image.FromFile("../../test_against.png", true);


            for (int x = 0; x < test.Width; x++)
            {
                for (int y = 0; y < test.Height; y++)
                {
                    Color c = test.GetPixel(x, y);
                    Vector3 color = new Vector3(c.R, c.G, c.B);
                    float maxVal = 0;
                    foreach (Histogram h in histograms)
                    {
                        float val = h.getValue(color);
                        if (val > maxVal)
                        {
                            maxVal = val;
                        }
                    }
                    maxVal *= 255;
                    if (maxVal !=0)
                        test.SetPixel(x, y, Color.FromArgb(255, (int)maxVal, 0, (int)maxVal));
                }
            }
            test.Save("../../test_against_output.png");


            return 0;
        }
    }
}
