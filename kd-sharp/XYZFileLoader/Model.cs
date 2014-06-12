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
        public const int DIMENSION = 20;
        private Brick[, ,] voxelGrid = new Brick[DIMENSION, DIMENSION, DIMENSION];
        private List<Point>[, ,] pointGrid = new List<Point>[DIMENSION, DIMENSION, DIMENSION];
        public List<LocatedBrick> bricks;

        public Model()
        {
            this.bricks = new List<LocatedBrick>();
            this.bricks.Add(new LocatedBrick(false, new Vector3(0, 0, 0),Color.Green));
            this.bricks.Add(new LocatedBrick(true, new Vector3(4, 1, -4), Color.Red));
            this.bricks.Add(new LocatedBrick(false, new Vector3(-1, 2, -1), Color.Blue));
            this.bricks.Add(new LocatedBrick(true, new Vector3(2, -1, -2), Color.Blue));
            this.bricks.Add(new LocatedBrick(false, new Vector3(-1, -2, 2), Color.Blue));
            
            //this.bricks.Add(new LocatedBrick(true, new Vector3(3, -1, -3)));
            //this.bricks.Add(new LocatedBrick(false, new Vector3(0, 2, 0)));
            //this.bricks.Add(new LocatedBrick(false, new Vector3(0, -2, 0)));

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

            for (int x = 0; x < voxelGrid.GetLength(0); x++)
            {
                for (int y = 0; y < voxelGrid.GetLength(1); y++)
                {
                    for (int z = 0; z < voxelGrid.GetLength(2); z++)
                    {
                        if (voxelGrid[x, y, z] == null)
                        {
                            continue;
                        }
                        else
                        {
                            
                            List<Point> newVoxel;

                            #region check Z
                            newVoxel = new List<Point>(pointGrid[x, y, z - 1]);
                            foreach (Point p in pointGrid[x, y, z - 1])
                            {
                                
                                if (p.normal.Z > 0)
                                {
                                    newVoxel.Remove(p);
                                }
                            }
                            pointGrid[x, y, z - 1] = newVoxel;

                            
                            newVoxel = new List<Point>(pointGrid[x, y, z + 1]);
                            foreach (Point p in pointGrid[x, y, z + 1])
                            {
                                
                                if (p.normal.Z < 0)
                                {
                                    newVoxel.Remove(p);
                                }
                            }
                            pointGrid[x, y, z + 1] = newVoxel;
                            #endregion
                            #region check X
                            newVoxel = new List<Point>(pointGrid[x-1, y, z]);
                            foreach (Point p in pointGrid[x-1, y, z])
                            {
                                
                                if (p.normal.X > 0)
                                {
                                    newVoxel.Remove(p);
                                }
                            }
                            pointGrid[x-1, y, z] = newVoxel;


                            newVoxel = new List<Point>(pointGrid[x+1, y, z]);
                            foreach (Point p in pointGrid[x+1, y, z])
                            {
                                
                                if (p.normal.X < 0)
                                {
                                    newVoxel.Remove(p);
                                }
                            }
                            pointGrid[x+1, y, z] = newVoxel;
                            #endregion
                            #region check Y
                            newVoxel = new List<Point>(pointGrid[x, y-1, z]);
                            foreach (Point p in pointGrid[x, y-1, z])
                            {
                                System.Diagnostics.Debug.WriteLine(p.normal);
                                if (p.normal.Y > 0)
                                {
                                    newVoxel.Remove(p);
                                }
                            }
                            pointGrid[x, y-1, z] = newVoxel;


                            newVoxel = new List<Point>(pointGrid[x, y+1, z]);
                            foreach (Point p in pointGrid[x, y+1, z])
                            {
                                
                                if (p.normal.Y < 0)
                                {
                                    newVoxel.Remove(p);
                                }
                            }
                            pointGrid[x, y+1, z] = newVoxel;
                            #endregion
                        }
                    }
                }
            }

            foreach (List<Point> l in pointGrid)
            {
                this.points.AddRange(l);
            }
        }
    }
}
