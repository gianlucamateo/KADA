using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Kinect;
using Point = XYZFileLoader.Point;

namespace KADA
{
    public class PipelineContainer
    {
        public int Stage;
        public DepthColor[,] DC;
        public Matrix R;
        public Matrix onlyRot;
        public List<Point> Qi;
        public List<Vector3> OutlierPoints;
        public Vector3 center, computedCenter;
        public DepthImagePixel[] DepthPixels;
        public byte[] ColorPixels;
        public Vector3[] NormalsList;
        public Vector3[,] Normals;
        public ColorImagePoint[] ColorPoints;
        public int ICPInliers = 0;
        public int ICPOutliers = 0;
        public float ICPRatio = 0f;
        public int Number;
        public List<DateTime> Timings;
        public Vector3[] modelVectors;
        public Vector3[] estimatedVectors;
        public Matrix rawNormalR;
        public Matrix normalR;
        public Vector3 outlierCenter;
        

        public PipelineContainer(PipelineDataContainer dataContainer)
        {
            this.outlierCenter = Vector3.Zero;
            this.modelVectors = new Vector3[3];
            this.rawNormalR = Matrix.Identity;
            this.normalR = Matrix.Identity;
            this.estimatedVectors = new Vector3[3];
            this.NormalsList = new Vector3[3];
            this.Timings = new List<DateTime>();
            this.Stage = 0;
            this.DC = new DepthColor[640, 480];
            this.R = Matrix.Identity;
            this.Qi = new List<Point>();
            this.OutlierPoints = new List<Vector3>();
            this.center = Vector3.Zero;
            this.computedCenter = Vector3.Zero;
            this.DepthPixels = new DepthImagePixel[dataContainer.DEPTHLENGTH];
            this.ColorPixels = new byte[dataContainer.COLORLENGTH];
            this.ColorPoints = new ColorImagePoint[640 * 480];
            this.Normals = new Vector3[640, 480];
        }
    }
}
