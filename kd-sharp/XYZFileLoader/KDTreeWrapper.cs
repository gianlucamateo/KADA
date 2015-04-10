using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using DotNumerics;
using Vector = DotNumerics.LinearAlgebra.Vector;

namespace KADA
{
    public class KDTreeWrapper
    {
        private KDTree.KDTree<Point> kdTree;
        public List<Point> points;
        public List<Vector> qi_prime;

        public KDTreeWrapper(){
            kdTree = new KDTree.KDTree<Point>(3,1);
            
            points = new List<Point>();
            qi_prime = new List<Vector>();
        }

        public KDTree.NearestNeighbor<Point> NearestNeighbors(double[] tSearchPoint, int iMaxReturned, double fDistance = -1)
        {
            return this.kdTree.NearestNeighbors(tSearchPoint, iMaxReturned, fDistance);
        }

        public void AddPoint(Vector3 position, Point data){
            double[] pos = {position.X, position.Y, position.Z};
            this.kdTree.AddPoint(pos, data);
            this.points.Add(data);
            Vector v = new Vector(pos);
            this.qi_prime.Add(v);
            //Console.WriteLine(""+position + "  "+data);
        }
    }
}
