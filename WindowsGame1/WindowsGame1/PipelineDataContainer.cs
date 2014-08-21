using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Model = XYZFileLoader.Model;

namespace KADA
{
    public enum TrackingConfidenceLevel
    {
        NONE, NORMALS, ICPTENTATIVE, ICPFULL
    }
    public class PipelineDataContainer
    {
        public int ICPInliers, ICPOutliers;
        public float ICPRatio;
        public readonly float NORMAL_CULLING_LIMIT = 0f;

        //public Vector3[] trackedNormals;

        public Matrix prevNormalR = Matrix.Identity;
        public float frameTime = 0, generateTime = 0;
        public DateTime lastTick = DateTime.Now, lastGeneration = DateTime.Now;
        public int COLORLENGTH, DEPTHLENGTH;
        public Model model;
        public bool generateBackground = false;
        public bool deNoiseAndICP = false;
        public int SLEEPTIME = 1;
        public int MINFRAMESINCONTAINER = 1;
        public bool run = true;
        public Matrix lastConfidentR = Matrix.Identity;
        public TrackingConfidenceLevel trackingConfidence = TrackingConfidenceLevel.NONE;
        public int[] normalMappings;
        public Vector3[] modelNormals;
        public BackgroundEvaluator backgroundEvaluator;

        public PipelineDataContainer()
        {
            this.backgroundEvaluator = new BackgroundEvaluator(this);
            this.modelNormals = new Vector3[3];
            this.model = new Model();
            this.normalMappings = new int[3];
            this.modelNormals[0] = Vector3.UnitX;
            this.modelNormals[1] = Vector3.UnitY;
            this.modelNormals[2] = Vector3.UnitZ;
            normalMappings[0] = 2;
            normalMappings[1] = 0;
            normalMappings[2] = 1;
            
        }

        public void recordTick()
        {
            this.frameTime = (DateTime.Now - this.lastTick).Milliseconds + frameTime;
            this.frameTime = frameTime / 2;
            this.lastTick = DateTime.Now;
        }
        public void recordGenerationTick()
        {
            this.generateTime = (DateTime.Now - this.lastGeneration).Milliseconds + generateTime;
            this.generateTime = generateTime / 2;
            this.lastGeneration = DateTime.Now;
        }

        public XYZFileLoader.KDTreeWrapper generateKDTree()
        {
            return model.generateKDTree();
        }
    }
}
