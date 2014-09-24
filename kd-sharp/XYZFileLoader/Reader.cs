using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{
    public enum BrickColor
    {
        NONE = 0,RED = 2,GREEN = 3,BLUE = 5,YELLOW = 7
    }
    public struct Point
    {
        public Vector3 position;
        public Vector3 normal;
        public double[] positionArr;
        public int brickColorInteger;
        public BrickColor brickColor;

        public Point(Vector3 pos, Vector3 nor)
        {
            position = pos;
            normal = nor;
            positionArr = new double[] { pos.X, pos.Y, pos.Z };
            brickColor = BrickColor.NONE;
            brickColorInteger = 1;
        }
        public Point Copy()
        {
            return new Point(this.position, this.normal);
        }
    }

    public class Reader
    {
        private const float SCALE = 1;
        public static List<Vector3> positions;
        public static List<Point> points;

        public static List<Point> getPoints()
        {
            if (points == null)
            {
                readFromFile();
            }
            return new List<Point>(points);
        }
        public static void readFromFile()
        {
            Random r = new Random();
            //Vector3 offset = new Vector3(-36.5f, -17f, 15f);
            positions = new List<Vector3>();
            points = new List<Point>();
            KDTreeWrapper kdTree;
            kdTree = new KDTreeWrapper();
            string[] lines = System.IO.File.ReadAllLines("ressources/pointcloud/Duplo_cleaned_2500samples_stratified_triangle.xyz");
            string[] parts;
            foreach (string line in lines)
            {
                parts = line.Split(' ');
                float x = float.Parse(parts[0]) * 10;
                float y = float.Parse(parts[1]) * 10;
                float z = float.Parse(parts[2]) * 10;
                Vector3 pos = new Vector3(x, y, z);
                Vector3 originalPos = pos;
                pos *= SCALE;
                //pos += offset*SCALE;

                /*Vector3 normal = new Vector3(float.Parse(parts[3]), float.Parse(parts[4]), float.Parse(parts[5]));
                normal.Normalize();*/

                Vector3 normal = Vector3.Zero;

                if (Math.Abs(originalPos.X - 0) < 0.01)
                {
                    normal = -Vector3.UnitX;
                }
                if (Math.Abs(originalPos.X - 64) < 0.01)
                {
                    normal = Vector3.UnitX;
                }

                if (Math.Abs(originalPos.Y - 19.2) < 0.01)
                {
                    normal = Vector3.UnitY;
                }

                if (Math.Abs(originalPos.Z + 0) < 0.01)
                {
                    normal = Vector3.UnitZ;
                }

                if (Math.Abs(originalPos.Z + 32) < 0.01)
                {
                    normal = -Vector3.UnitZ;
                }

                Point p = new Point(pos, normal);

                //if (Math.Abs(originalPos.X - 1.5f) > 0.01 && Math.Abs(originalPos.X - 62.5f) > 0.01 && Math.Abs(originalPos.Y - 17.7f) > 0.01 && Math.Abs(originalPos.Z + 30.5f) > 0.01 && Math.Abs(originalPos.Z + 1.5f) > 0.01)
                if (normal != Vector3.Zero)
                {
                    if (r.NextDouble() > 0)
                    {
                        kdTree.AddPoint(pos, p);
                        points.Add(p);
                        positions.Add(pos);
                    }
                }
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
            //return kdTree;
        }
    }
}
