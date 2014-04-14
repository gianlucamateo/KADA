using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{
    public struct Point
    {
        Vector3 position;
        Vector3 normal;
        public Point(Vector3 pos, Vector3 nor)
        {
            position = pos;
            normal = nor;
        }
    }
    
    public class Reader
    {
        public static KDTree.KDTree<Point> readFromFile(Vector3 offset)
        {
            KDTree.KDTree<Point> kdTree;
            kdTree = new KDTree.KDTree<Point>(3);
            string[] lines = System.IO.File.ReadAllLines("../../ressources/pointcloud/Duplo_5000samples.xyz");
            string[] parts;
            foreach (string line in lines)
            {
                parts = line.Split(' ');
                float x = float.Parse(parts[0])*10;
                float y = float.Parse(parts[1])*10;
                float z = float.Parse(parts[2])*10;
                Vector3 pos = new Vector3(x, y, z);
                pos -= offset;
                Point p = new Point(pos, new Vector3(float.Parse(parts[3]), float.Parse(parts[4]), float.Parse(parts[5])));
                double[] position = {x,y,z};
                kdTree.AddPoint(position,p);                
            }
            return kdTree;
        }
    }
}
