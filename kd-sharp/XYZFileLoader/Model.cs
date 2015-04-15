using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using System.Threading.Tasks;
using System.Collections.Concurrent;

namespace KADA
{

    public class Model
    {

        public List<Point> points;
        public const int DIMENSION = 26;
        public Brick[, ,] voxelGrid = new Brick[DIMENSION, DIMENSION, DIMENSION];
        public List<Point>[, ,] pointGrid = new List<Point>[DIMENSION, DIMENSION, DIMENSION];//, newGrid = new List<Point>[DIMENSION, DIMENSION, DIMENSION];
        public List<LocatedBrick> Bricks;
        public KDTreeWrapper kdTree;
        public float radius;
        public Vector3 center;
        public List<TentativeModel> tentativeModels;

        public Model(bool definitive, Vector3 center, List<LocatedBrick> bricks = null, bool fast = false)
        {
            this.center = center;
            if (bricks == null)
            {
                this.Bricks = new List<LocatedBrick>();
                this.Bricks.Add(new LocatedBrick(false, new Vector3(0, 0, 0), BrickColor.GREEN));
                this.Bricks.Add(new LocatedBrick(true, new Vector3(4, 1, -4), BrickColor.RED));
                this.Bricks.Add(new LocatedBrick(true, new Vector3(2, -1, -2), BrickColor.BLUE));
                this.Bricks.Add(new LocatedBrick(false, new Vector3(-1, -2, 2), BrickColor.BLUE));
                this.Bricks.Add(new LocatedBrick(false, new Vector3(3, 2, -1), BrickColor.GREEN));
            }
            else
            {
                this.Bricks = new List<LocatedBrick>(bricks);
            }

            //this.bricks.Add(new LocatedBrick(true, new Vector3(3, -1, -3)));
            //this.bricks.Add(new LocatedBrick(false, new Vector3(0, 2, 0)));
            //this.bricks.Add(new LocatedBrick(false, new Vector3(0, -2, 0)));

            this.tentativeModels = new List<TentativeModel>();

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

            /*for (int x = 0; x < voxelGrid.GetLength(0); x++)
            {
                for (int y = 0; y < voxelGrid.GetLength(1); y++)
                {
                    for (int z = 0; z < voxelGrid.GetLength(2); z++)
                    {
                        newGrid[x, y, z] = new List<Point>();
                    }
                }
            }*/

            this.points = new List<Point>();
            //points = Reader.getPoints();
            if (definitive)
            {
                DateTime before = DateTime.Now;
                GenerateKDTree(fast);
                if (!fast)
                {
                    Console.WriteLine("GenerateKDTree took: " + (DateTime.Now - before));
                }
                if (!fast)
                {
                    before = DateTime.Now;
                    ComputeTentativeBricks();
                    Console.WriteLine("computing tentative models took: " + (DateTime.Now - before));
                }
            }
        }

        public void ComputeTentativeBricks()
        {
            ConcurrentBag<TentativeModel> tModels = new ConcurrentBag<TentativeModel>();
            
            for (int y = -(DIMENSION / 2 + 1); y < (DIMENSION / 2 + 1); y++)
            {

                for (int x = -(DIMENSION / 2 + 1); x < (DIMENSION / 2 + 1); x++)
                {
                    //for (int z = -(DIMENSION / 2 + 1); z < (DIMENSION / 2 + 1); z++)
                    //{
                    Parallel.For(-(DIMENSION / 2 + 1), (DIMENSION / 2 + 1), new ParallelOptions { MaxDegreeOfParallelism = 10 }, z =>
                    {
                        Vector3 vOffset = new Vector3(x, y, z);
                        bool tryInsert = false;
                        foreach (LocatedBrick b in this.Bricks)
                        {
                            Vector3 diff = (b.voxelOffset-vOffset);
                            if(diff.Length()<17&&Math.Abs(diff.Y)<2f){
                                tryInsert = true;
                            }
                        }
                        //tryInsert = true;
                        if (tryInsert)
                        {
                            LocatedBrick tentativeBrick = new LocatedBrick(true, new Vector3(x, y, z), BrickColor.GREEN);
                            if (tentativeBrick.insert(this.pointGrid, this.voxelGrid, false))
                            {
                                //this.tentativeModels.Add(new TentativeModel(this.Bricks, tentativeBrick, this.center));
                                tModels.Add(new TentativeModel(this.Bricks, tentativeBrick, this.center));
                            }
                            tentativeBrick = new LocatedBrick(false, new Vector3(x, y, z), BrickColor.GREEN);
                            if (tentativeBrick.insert(this.pointGrid, this.voxelGrid, false))
                            {
                                //this.tentativeModels.Add(new TentativeModel(this.Bricks, tentativeBrick, this.center));
                                tModels.Add(new TentativeModel(this.Bricks, tentativeBrick, this.center));
                            }
                        }
                    });
                }
            }
            this.tentativeModels.AddRange(tModels);
            
        }

        private void Reset()
        {
            this.voxelGrid = new Brick[DIMENSION, DIMENSION, DIMENSION];
            this.pointGrid = new List<Point>[DIMENSION, DIMENSION, DIMENSION];
            this.points = new List<Point>();
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
            /*for (int x = 0; x < voxelGrid.GetLength(0); x++)
            {
                for (int y = 0; y < voxelGrid.GetLength(1); y++)
                {
                    for (int z = 0; z < voxelGrid.GetLength(2); z++)
                    {
                        newGrid[x, y, z]=new List<Point>();
                    }
                }
            }*/
        }
        public KDTreeWrapper GenerateKDTree(bool fast = false)
        {
            this.Reset();
            foreach (LocatedBrick b in Bricks)
            {
                b.insert(this.pointGrid, this.voxelGrid, true);
            }
            this.kdTree = new KDTreeWrapper();

            updatePoints(fast);


            foreach (Point p in this.points)
            {
                if (p.normal != Vector3.Zero && p.position != Vector3.Zero)
                {
                    kdTree.AddPoint(p.position, p);
                }

            }
            return kdTree;
        }
        public KDTreeWrapper getKDTree()
        {
            return this.kdTree;
        }
        private void updatePoints(bool fast = false)
        {
            this.points.Clear();
            if (!fast)
            {

                /*for (int x = 0; x < voxelGrid.GetLength(0); x++)
                {
                    for (int y = 0; y < voxelGrid.GetLength(1); y++)
                    {
                        for (int z = 0; z < voxelGrid.GetLength(2); z++)
                        {
                            newGrid[x, y, z] = new List<Point>();
                        }
                    }
                } */
                //int blockCount = 0;
                for (int x = 0; x < voxelGrid.GetLength(0); x++)
                {
                    for (int y = 0; y < voxelGrid.GetLength(1); y++)
                    {
                        for (int z = 0; z < voxelGrid.GetLength(2); z++)
                        {
                            //Console.WriteLine(blockCount++);
                            if (voxelGrid[x, y, z] == null)
                            {
                                continue;
                            }
                            else
                            {
                                #region check Z


                                foreach (Point p in pointGrid[x, y, z - 1])
                                {
                                    if (p.normal.Z > 0)
                                    {
                                        p.state.dismiss = true;

                                    }
                                }



                                foreach (Point p in pointGrid[x, y, z + 1])
                                {
                                    if (p.normal.Z < 0)
                                    {
                                        p.state.dismiss = true;
                                    }
                                }

                                #endregion
                                #region check X
                                if (x > 0)
                                {

                                    foreach (Point p in pointGrid[x - 1, y, z])
                                    {
                                        if (p.normal.X > 0)
                                        {
                                            p.state.dismiss = true;
                                        }
                                    }

                                }


                                if (x < voxelGrid.GetLength(0) - 1)
                                {

                                    foreach (Point p in pointGrid[x + 1, y, z])
                                    {
                                        if (p.normal.X < 0)
                                        {
                                            p.state.dismiss = true;
                                        }
                                    }

                                }
                                #endregion
                                #region check Y
                                if (y > 0)
                                {

                                    foreach (Point p in pointGrid[x, y - 1, z])
                                    {
                                        if (p.normal.Y > 0)
                                        {
                                            p.state.dismiss = true;
                                        }
                                    }

                                }

                                if (y < voxelGrid.GetLength(1) - 1)
                                {


                                    foreach (Point p in pointGrid[x, y + 1, z])
                                    {
                                        if (p.normal.Y < 0)
                                        {
                                            p.state.dismiss = true;
                                        }
                                    }

                                }
                                #endregion
                            }
                        }
                    }
                }


                /*for (int x = 0; x < voxelGrid.GetLength(0); x++)
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
                                if (x > 0)
                                {
                                    newVoxel = new List<Point>(pointGrid[x - 1, y, z]);
                                    foreach (Point p in pointGrid[x - 1, y, z])
                                    {

                                        if (p.normal.X > 0)
                                        {
                                            newVoxel.Remove(p);
                                        }
                                    }
                                    pointGrid[x - 1, y, z] = newVoxel;
                                }

                                if (x < voxelGrid.GetLength(0) - 1)
                                {
                                    newVoxel = new List<Point>(pointGrid[x + 1, y, z]);
                                    foreach (Point p in pointGrid[x + 1, y, z])
                                    {

                                        if (p.normal.X < 0)
                                        {
                                            newVoxel.Remove(p);
                                        }
                                    }
                                    pointGrid[x + 1, y, z] = newVoxel;
                                }
                                #endregion
                                #region check Y
                                if (y > 0)
                                {
                                    newVoxel = new List<Point>(pointGrid[x, y - 1, z]);
                                    foreach (Point p in pointGrid[x, y - 1, z])
                                    {
                                        // System.Diagnostics.Debug.WriteLine(p.normal);
                                        if (p.normal.Y > 0)
                                        {
                                            newVoxel.Remove(p);
                                        }
                                    }
                                    pointGrid[x, y - 1, z] = newVoxel;
                                }

                                if (y < voxelGrid.GetLength(1) - 1)
                                {
                                    newVoxel = new List<Point>(pointGrid[x, y + 1, z]);
                                    foreach (Point p in pointGrid[x, y + 1, z])
                                    {

                                        if (p.normal.Y < 0)
                                        {
                                            newVoxel.Remove(p);
                                        }
                                    }
                                    pointGrid[x, y + 1, z] = newVoxel;
                                }
                                #endregion
                            }
                        }
                    }
                }*/
            }
            foreach (List<Point> l in pointGrid)
            {
                foreach (Point p in l)
                {
                    if (p.state.dismiss == false)
                    {
                        this.points.Add(p);
                    }
                }

            }
            //this.center = Vector3.Zero;

            //compute center
            if (this.center == Vector3.Zero)
            {
                foreach (Point p in this.points)
                {
                    this.center += p.position;
                }
                this.center /= points.Count;
            }

            //compute max Distance, radius of wrapping sphere
            this.radius = 0f;

            //if (!fast)
            //{
            for (int i = 0; i < this.points.Count; i++)
            {
                Point p = this.points[i];
                p.position = p.position - this.center;
                this.points[i] = p;
            }
            //}

            foreach (Point p in this.points)
            {
                float length = (p.position - this.center).Length();
                if (length > radius)
                {
                    radius = length;
                }
            }

        }
    }
}
