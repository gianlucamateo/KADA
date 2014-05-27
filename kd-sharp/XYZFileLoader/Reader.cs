using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{
    public struct Point
    {
        public Vector3 position;
        public Vector3 normal;
        public double[] positionArr;
        public Point(Vector3 pos, Vector3 nor)
        {
            position = pos;
            normal = nor;
            positionArr = new double[] { pos.X, pos.Y, pos.Z };
        }
    }

    public class Reader
    {
        public static List<Vector3> positions;
        public static KDTreeWrapper readFromFile(Vector3 offset)
        {
            positions = new List<Vector3>();
            KDTreeWrapper kdTree;
            kdTree = new KDTreeWrapper();
            string[] lines = System.IO.File.ReadAllLines("../../ressources/pointcloud/Duplo_cleaned_2500samples_stratified_triangle.xyz");
            string[] parts;
            foreach (string line in lines)
            {
                parts = line.Split(' ');
                float x = float.Parse(parts[0]) * 10;// *2;
                float y = float.Parse(parts[1]) * 10;// *2;
                float z = float.Parse(parts[2]) * 10;// *2;
                Vector3 pos = new Vector3(x, y, z);
                pos += offset;// *2;
                positions.Add(pos);
                Point p = new Point(pos, new Vector3(float.Parse(parts[3]), float.Parse(parts[4]), float.Parse(parts[5])));
                kdTree.AddPoint(pos, p);
                /*
                pos = new Vector3(x, y, z);
                pos += offset;
                pos += new Vector3(0,-18,0);
                positions.Add(pos);
                p = new Point(pos, new Vector3(float.Parse(parts[3]), float.Parse(parts[4]), float.Parse(parts[5])));
                kdTree.AddPoint(pos, p);
                pos = new Vector3(x, y, z);
                pos += offset;
                pos += new Vector3(0, -18, 0);
                positions.Add(pos);
                p = new Point(pos, new Vector3(float.Parse(parts[3]), float.Parse(parts[4]), float.Parse(parts[5])));
                kdTree.AddPoint(pos, p);*/
            }
            return kdTree;
        }
    }
}
