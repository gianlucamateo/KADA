using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{
    public class KDTreeWrapper
    {
        private KDTree.KDTree<Point> kdTree;
        public List<Point> points;

        public KDTreeWrapper(){
            kdTree = new KDTree.KDTree<Point>(3);
            points = new List<Point>();
        }

        public KDTree.NearestNeighbour<Point> NearestNeighbors(double[] tSearchPoint, int iMaxReturned, double fDistance = -1)
        {
            return this.kdTree.NearestNeighbors(tSearchPoint, iMaxReturned, fDistance);
        }

        public void AddPoint(Vector3 position, Point data){
            double[] pos = {position.X, position.Y, position.Z};
            this.kdTree.AddPoint(pos, data);
            this.points.Add(data);
        }
    }
}
