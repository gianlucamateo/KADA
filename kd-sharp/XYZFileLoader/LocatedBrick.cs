using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{

    public class LocatedBrick
    {
        private Matrix Transformation;
        public bool rotated;
        public Vector3 voxelOffset;
        private Brick brick;
        private Vector3 root = new Vector3(Model.DIMENSION / 2, Model.DIMENSION / 2, Model.DIMENSION / 2);
        private Vector3 voxelDimensions = new Vector3(64.0f / 4, 19.2f, 32.0f / 2);

        public LocatedBrick(bool rotated, Vector3 voxelOffset)
        {
            this.rotated = rotated;
            this.voxelOffset = voxelOffset;
            this.brick = new Brick();
            if (rotated)
            {
                float save = this.voxelOffset.X;
                this.voxelOffset.X = this.voxelOffset.Z;
                this.voxelOffset.Z = save;
            }
            this.Transformation = Matrix.Identity;
        }

        public Matrix getTransformation()
        {
            return this.Transformation;
        }

        public void insert(List<Point>[, ,] pointGrid, Brick[, ,] voxelGrid)
        {
            
            Vector3 translation = new Vector3();
            translation.X = voxelDimensions.X * voxelOffset.X;
            translation.Y = voxelDimensions.Y * voxelOffset.Y;
            translation.Z = voxelDimensions.Z * voxelOffset.Z;

            if (this.rotated)
            {
                this.Transformation = Matrix.Multiply(this.Transformation, Matrix.CreateRotationY((float)Math.PI / 2));
                translation.X += 6*voxelDimensions.X;
                translation.Z -= 2 * voxelDimensions.Z;
            }


            this.Transformation = Matrix.Multiply(this.Transformation, Matrix.CreateTranslation(translation));
            for (int BrickX = 0; BrickX < 4; BrickX++)
            {
                for (int BrickY = 0; BrickY > -2; BrickY--)
                {
                    if (rotated == false)
                    {
                        voxelGrid[(int)voxelOffset.X + BrickX + (int)root.X, (int)voxelOffset.Y + (int)root.Y, (int)voxelOffset.Z + BrickY + (int)root.Z] = this.brick;
                    }
                    else
                    {
                        int x = (int)voxelOffset.Z + BrickY + (int)root.X-1;
                        int y = (int)voxelOffset.Y + (int)root.Y;
                        int z = (int)voxelOffset.X + BrickX + (int)root.Z+1;
                        voxelGrid[x, y, z] = this.brick;
                    }
                }
            }
            foreach (Point p in this.brick.points)
            {

                Vector3 position = p.position;

                /*float swap = position.Y;
                position.Y = position.Z;
                position.Z = swap;*/

                int x = (int)(position.X / voxelDimensions.X);

                x = Math.Min(x, 3);
                x += (int)voxelOffset.X + (int)root.X;

                int y = (int)voxelOffset.Y + (int)root.Y;
                int z = (int)(-position.Z / voxelDimensions.Z);


                z = Math.Min(z, 1);

                z *= -1;
                z += (int)voxelOffset.Z + (int)root.Z;
                /*int x = Math.Min((int)(p.position.X / voxelDimensions.X), 3) + (int)voxelOffset.X + (int)root.X;
                int z = Math.Min((int)(p.position.Y / voxelDimensions.Z), 1) + (int)voxelOffset.Z + (int)root.Z;
                int y = (int)voxelOffset.Y + (int)root.Y;*/
                if (rotated)
                {
                    int save = x;
                    x = z;
                    z = save;
                    x--;
                    z++;
                }

                Point pCopy = p.Copy();
                pCopy.position.X += voxelOffset.X * voxelDimensions.X;
                pCopy.position.Y += voxelOffset.Y * voxelDimensions.Y;
                pCopy.position.Z += voxelOffset.Z * voxelDimensions.Z;
                if (rotated)
                {
                    float save = pCopy.position.X;
                    pCopy.position.X = pCopy.position.Z;
                    pCopy.position.Z = save;

                    save = pCopy.normal.X;
                    pCopy.normal.X = pCopy.normal.Z;
                    pCopy.normal.Z = save;
                }
                Vector3 offset = new Vector3(-36.5f, -17f, 15f);
                pCopy.position += offset;
                pointGrid[x, y, z].Add(pCopy);


            }

        }
    }
}
