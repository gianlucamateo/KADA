using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{

    class LocatedBrick
    {
        public bool rotated;
        public Vector3 voxelOffset;
        private Brick brick;
        private Vector3 root = new Vector3(20, 20, 20);
        private Vector3 voxelDimensions = new Vector3(63.0f / 4, 31.5f / 2, 19);

        public LocatedBrick(bool rotated, Vector3 voxelOffset)
        {
            this.rotated = rotated;
            this.voxelOffset = voxelOffset;
            this.brick = new Brick();
        }

        public void insert(List<Point>[, ,] pointGrid, Brick[, ,] voxelGrid)
        {
            for (int BrickX = 0; BrickX < 4; BrickX++)
            {
                for (int BrickY = 0; BrickY < 2; BrickY++)
                {
                    if (rotated == false)
                    {
                        voxelGrid[(int)voxelOffset.X + BrickX, (int)voxelOffset.Y + BrickY, (int)voxelOffset.Z] = this.brick;
                    }
                    else
                    {
                        voxelGrid[(int)voxelOffset.X + BrickY, (int)voxelOffset.Y + BrickX, (int)voxelOffset.Z] = this.brick;
                    }
                }
            }
            foreach (Point p in this.brick.points)
            {

                //TODO: ROTATION
                int x = Math.Max((int)(p.position.X / voxelDimensions.X),3) + (int)voxelOffset.X;
                int y = Math.Max((int)(p.position.Y / voxelDimensions.Y), 1) + (int)voxelOffset.Y;
                int z = (int)voxelOffset.Z;
                Point pCopy = p.Copy();
                pCopy.position.X += voxelOffset.X * voxelDimensions.X;
                pCopy.position.Y += voxelOffset.Y * voxelDimensions.Y;
                pCopy.position.Z += voxelOffset.Z * voxelDimensions.Z;
                pointGrid[x, y, z].Add(pCopy); 
                
            }

        }
    }
}
