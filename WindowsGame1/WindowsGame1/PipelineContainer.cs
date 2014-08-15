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
        public Matrix onlyRot;
        public List<Vector3> qi;
        public Vector3 center;
        public DepthImagePixel[] depthPixels;
        public byte[] colorPixels;
        public Vector3[] Normals;
        public ColorImagePoint[] colorPoints;
        public int ICPInliers = 0;
        public int ICPOutliers = 0;
        public float ICPRatio = 0f;
        public int number;
        public List<DateTime> timings;
        public Vector3[] modelVectors;
        public Vector3[] estimatedVectors;
        public Matrix normalR;
        

        public PipelineContainer(PipelineDataContainer dataContainer)
        {
            this.modelVectors = new Vector3[3];
            this.normalR = Matrix.Identity;
            this.estimatedVectors = new Vector3[3];
            this.Normals = new Vector3[3];
            this.timings = new List<DateTime>();
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
