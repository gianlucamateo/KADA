using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Model = XYZFileLoader.Model;

namespace KADA
{
    public class PipelineDataContainer
    {
        public double ICPInliers, ICPOutliers, ICPRatio;
        public readonly float NORMAL_CULLING_LIMIT = 0f;
        public Vector3[] Normals = new Vector3[3];
        public Vector3 mostConfidentNormal = Vector3.UnitZ;
        public float frameTime = 0;
        public DateTime lastTick = DateTime.Now;
        public int COLORLENGTH, DEPTHLENGTH;
        public Model model;

        public PipelineDataContainer()
        {
            this.model = new Model();
            this.model.getKDTree();
        }

        public void recordTick()
        {
            this.frameTime = (DateTime.Now - this.lastTick).Milliseconds + frameTime;
            this.frameTime = frameTime / 2;
            this.lastTick = DateTime.Now;
        }

        public XYZFileLoader.KDTreeWrapper getKDTree()
        {
            return model.getKDTree();
        }
    }
}
