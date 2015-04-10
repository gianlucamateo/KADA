﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Model = KADA.Model;

namespace KADA
{
    public enum TrackingConfidenceLevel
    {
        NONE, NORMALS, ICPTENTATIVE, ICPFULL
    }
    public class PipelineDataContainer
    {
        public int ICPInliers, ICPOutliers;
        public float ICPMSE;
        public readonly float NORMAL_CULLING_LIMIT = 0f;

        //public Vector3[] trackedNormals;

        public Boolean UseYUV = true;
        public Matrix prevNormalR = Matrix.Identity;
        public Matrix normalRotationAdjustment = Matrix.Identity;
        public float frameTime = 0, generateTime = 0;
        public DateTime lastTick = DateTime.Now, lastGeneration = DateTime.Now;
        public int COLORLENGTH, DEPTHLENGTH;
        public Model model;
        public SortedDictionary<float,TentativeModel> tentativeModels;
        public bool GenerateBackground = false;
        public bool DeNoiseAndICP = false;
        public int SLEEPTIME = 1;
        public int MINFRAMESINCONTAINER = 1;
        public bool Run = true;
        public Matrix lastConfidentR = Matrix.Identity;
        public TrackingConfidenceLevel trackingConfidence = TrackingConfidenceLevel.NONE;
        public int[] normalMappings;
        public Queue<Vector3> outlierCenters;
        public Vector3 outlierCenter;
        public Vector3[] modelNormals;
        public Vector3 center;
        public BackgroundEvaluator backgroundEvaluator;
        public Matrix R;
        public bool EditMode;
        public bool Attach;
        public bool WrongModel=false;
        public bool ApplyModel = false;
        public int ModelsWorked = 0;
        public Vector3 g = Vector3.Zero;
        public Matrix BaseRotation;
        public BrickColor addColor = BrickColor.NONE;
        public float currentMaxMSE;
        public int differentViewCounter = 0;
        public bool RevertToOld = false;
        public List<Point> comparisonPoints;

        public float ICPThreshold = 200;

        public PipelineDataContainer()
        {
            this.comparisonPoints = new List<Point>();
            this.BaseRotation = Matrix.Identity;
            this.Attach = false;
            this.EditMode = false;
            this.R = new Matrix();
            this.center = Vector3.Zero;
            this.outlierCenter = Vector3.Zero;
            this.outlierCenters = new Queue<Vector3>();
            this.backgroundEvaluator = new BackgroundEvaluator(this);
            this.modelNormals = new Vector3[3];
            this.model = new Model(true,Vector3.Zero);
            //this.tentativeModel = new TentativeModel(model.Bricks,new LocatedBrick(false,Vector3.Zero,BrickColor.NONE));
            this.normalMappings = new int[3];
            this.modelNormals[0] = Vector3.UnitX;
            this.modelNormals[1] = Vector3.UnitY;
            this.modelNormals[2] = Vector3.UnitZ;
            normalMappings[0] = 0;
            normalMappings[1] = 1;
            normalMappings[2] = 2;
            
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

        
    }
}
