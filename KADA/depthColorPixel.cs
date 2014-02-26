using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KADA
{
    public struct DepthColorPixel
    {

        public DepthColorPixel(byte[] redV, byte[] greenV, byte[] blueV, double depthV = 0)
        {
            /* red = BitConverter.ToUInt32(redV, 0);
             green = BitConverter.ToUInt32(redV, 0);
             blue = BitConverter.ToUInt32(redV, 0);*/
            this.red = new byte[4];
            this.green = new byte[4];
            this.blue = new byte[4];

            Array.Copy(red, redV, 0);
            Array.Copy(green, greenV, 0);
            Array.Copy(blue, blueV, 0);

        }
        private byte[] red;
        public byte[] r
        {
            get
            {
                return red;
            }
        }
        private byte[] green;
        public byte[] g
        {
            get
            {
                return green;
            }
        }
        private byte[] blue;
        public byte[] b
        {
            get
            {
                return blue;
            }
        }
        public byte[] getData()
        {

            byte[] data = new byte[32];
            if (this.red != null)
            {
                byte[] colors;
                colors = this.red;
                //colors = BitConverter.GetBytes(red);
                //colors.Reverse();
                for (int i = 0; i < 4; i++)
                {
                    data[i] = colors[i];
                }
                colors = this.green;
                //colors = BitConverter.GetBytes(blue);
                //colors.Reverse();
                for (int i = 0; i < 4; i++)
                {
                    data[i + 4] = colors[i];
                }
                colors = this.blue;
                //colors = BitConverter.GetBytes(green);
                //colors.Reverse();
                for (int i = 0; i < 4; i++)
                {
                    data[i + 8] = colors[i];
                }

                for (int i = 12; i < data.Length; i++)
                {
                    data[i] = 0;
                }
            }
            return data;
        }


    }
}
