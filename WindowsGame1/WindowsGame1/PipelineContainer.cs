using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Kinect;

namespace KADA
{
    public class PipelineContainer
    {
        public int stage;
        public DepthColor[,] dc;
        public Matrix R;
        public List<Vector3> qi;
        public Vector3 center;
        public DepthImagePixel[] depthPixels;
        public byte[] colorPixels;
        public ColorImagePoint[] colorPoints;

        public PipelineContainer(PipelineDataContainer dataContainer)
        {
            this.stage = 0;
            this.dc = new DepthColor[640, 480];
            this.R = Matrix.Identity;
            this.qi = new List<Vector3>();
            this.center = Vector3.Zero;
            this.depthPixels = new DepthImagePixel[dataContainer.DEPTHLENGTH];
            this.colorPixels = new byte[dataContainer.COLORLENGTH];
            this.colorPoints = new ColorImagePoint[640 * 480];
        }
    }
}
