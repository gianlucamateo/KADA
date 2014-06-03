using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{

    public class Model
    {
        public List<Point> points;
        private Brick[, ,] voxelGrid = new Brick[40, 40, 40];
        private List<Point>[, ,] pointGrid = new List<Point>[40, 40, 40];
        private List<LocatedBrick> bricks;

        public Model()
        {
            this.bricks = new List<LocatedBrick>();
            this.bricks.Add(new LocatedBrick(false, new Vector3(0, 0, 0)));


            for (int x = 0; x < voxelGrid.GetLength(0); x++)
            {
                for (int y = 0; y < voxelGrid.GetLength(1); y++)
                {
                    for (int z = 0; z < voxelGrid.GetLength(2); z++)
                    {
                        pointGrid[x, y, z] = new List<Point>();
                    }
                }
            }
            this.points = new List<Point>();
            //points = Reader.getPoints();
        }

        public KDTreeWrapper getKDTree()
        {
            foreach (LocatedBrick b in bricks)
            {
                b.insert(this.pointGrid, this.voxelGrid);
            }
            KDTreeWrapper kdTree = new KDTreeWrapper();

            updatePoints();

            foreach (Point p in this.points)
            {
                if (p.normal != Vector3.Zero)
                {
                    kdTree.AddPoint(p.position, p);
                }
            }
            return kdTree;
        }
        private void updatePoints()
        {
            this.points.Clear();
            foreach (List<Point> l in pointGrid)
            {
                this.points.AddRange(l);
            }
        }
    }
}
