using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Text;

namespace KADA
{
    class ColorDepthMerger
    {

        public static void mergeFillBuffer(byte[] colorPixels, DepthColorPixel[,] cdMap, byte[] combinedPixels)
        {
            merge(colorPixels, cdMap);
            fillBuffer(cdMap, combinedPixels);
        }

        static void merge(byte[] colorPixels, DepthColorPixel[,] cdMap)
        {
            byte[] red, green, blue;
            red = new byte[4];
            green = new byte[4];
            blue = new byte[4];
            for (int i = 0; i < colorPixels.Length / 16; i++)
            {
                int x = i % 640;
                int y = i / 640;
                int index = i;
                Array.Copy(colorPixels,i,red,0,4);
                Array.Copy(colorPixels,i+4,green,0,4);
                Array.Copy(colorPixels,i+8,blue,0,4);
                cdMap[x, y] = new DepthColorPixel(red, green, blue);
                //System.Diagnostics.Debug.WriteLine(""+x+y);
            }

        }
        static void fillBuffer(DepthColorPixel[,] cdMap, byte[] combinedPixels)
        {
            byte[] data = new byte[16];
            for (int x = 0; x < cdMap.GetLength(0); x++)
            {
                for (int y = 0; y < cdMap.GetLength(1); y++)
                {
                    data = cdMap[x,y].getData();
                    int baseindex = x+y;
                    for (int i = 0; i < 16; i++)
                    {
                        combinedPixels[baseindex + i] = data[i];
                    }
                }
            }
        }
    }

}
