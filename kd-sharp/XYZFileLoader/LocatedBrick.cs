using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using System.Threading.Tasks;

namespace KADA
{

    public class LocatedBrick
    {
        private Matrix Transformation;
        public bool rotated;
        public Vector3 voxelOffset;
        public Brick brick;
        private Vector3 root = new Vector3(Model.DIMENSION / 2, Model.DIMENSION / 2, Model.DIMENSION / 2);
        private Vector3 voxelDimensions = new Vector3(64.0f / 4, 19.2f, 32.0f / 2);
        public Vector3 center;
        private BrickColor color;

        public LocatedBrick(bool rotated, Vector3 voxelOffset, BrickColor color)
        {
            this.color = color;
            this.rotated = rotated;
            this.voxelOffset = voxelOffset;
            this.brick = new Brick(color);
            if (rotated)
            {
                float save = this.voxelOffset.X;
                this.voxelOffset.X = this.voxelOffset.Z;
                this.voxelOffset.Z = save;
            }
            this.Transformation = Matrix.Identity;
        }

        public void setColor(BrickColor color)
        {
            this.color = color;
            this.brick.color = color;
        }

        public Matrix getTransformation()
        {
            return this.Transformation;
        }

        public Vector3 getColor()
        {
            Vector3 color = Vector3.Zero;
            switch (this.brick.color)
            {
                case BrickColor.BLUE:
                    color = new Vector3(0, 0, 1);
                    break;
                case BrickColor.RED:
                    color = new Vector3(1, 0, 0);
                    break;
                case BrickColor.GREEN:
                    color = new Vector3(0, 1, 0.2f);
                    break;
                case BrickColor.YELLOW:
                    color = new Vector3(1, 1, 0);
                    break;
            }
            return color;   
            //Vector3 val = new Vector3(this.brick.color.R, this.brick.color.G, this.brick.color.B);
            //return val/255f;
        }
        Brick[, ,] tentative;
        public bool insert(List<Point>[, ,] pointGrid, Brick[, ,] voxelGrid,bool apply)
        {
            if (tentative == null)
            {
                tentative = new Brick[Model.DIMENSION, Model.DIMENSION, Model.DIMENSION];
            }
            if (voxelGrid == null)
            {
                return false;
            }
            Array.Copy(voxelGrid, tentative, voxelGrid.GetLength(0) * voxelGrid.GetLength(1) * voxelGrid.GetLength(2));

            this.Transformation = Matrix.Identity;
            Vector3 translation = new Vector3();
            translation.X = voxelDimensions.X * voxelOffset.X;
            translation.Y = voxelDimensions.Y * voxelOffset.Y;
            translation.Z = voxelDimensions.Z * voxelOffset.Z;

            if (this.rotated)
            {
                this.Transformation = Matrix.Multiply(this.Transformation, Matrix.CreateRotationY((float)Math.PI / 2));
                
                float save = translation.X;
                translation.X = translation.Z;
                translation.Z = save;
                //translation.X += 8 * voxelDimensions.X;
                translation.Z += 4 * voxelDimensions.Z;
            }


            this.Transformation = Matrix.Multiply(this.Transformation, Matrix.CreateTranslation(translation));

            int lowerConnections = 0, upperConnections = 0;
            for (int BrickX = 0; BrickX < 4; BrickX++)
            {
                for (int BrickY = 0; BrickY > -2; BrickY--)
                {
                    if (rotated == false)
                    {

                        int x = (int)voxelOffset.X + BrickX + (int)root.X;
                        int y = (int)voxelOffset.Y + (int)root.Y;
                        int z = (int)voxelOffset.Z + BrickY + (int)root.Z;

                        if (x < 0 || y < 0 || z < 0 ||
                            x >= tentative.GetLength(0) || y >= tentative.GetLength(1) || z >= tentative.GetLength(2))
                        {
                            return false;
                        }
                        Brick b = tentative[x, y, z];
                        if (b == null)
                        {
                            tentative[x, y, z] = this.brick;                            
                        }
                        else
                        {
                            return false;
                        }
                        if (y > 0)
                        {
                            if (tentative[x, y - 1, z] != null)
                            {
                                lowerConnections++;
                            }
                        }
                        if (y < tentative.GetLength(1)-1)
                        {
                            if (tentative[x, y + 1, z] != null)
                            {
                                upperConnections++;
                            }
                        }
                    }
                    else
                    {
                        int x = (int)voxelOffset.Z + BrickY + (int)root.X-1;
                        int y = (int)voxelOffset.Y + (int)root.Y;
                        int z = (int)voxelOffset.X + BrickX + (int)root.Z+1;

                        if (x < 0 || y < 0 || z < 0 ||
                            x >= tentative.GetLength(0) || y >= tentative.GetLength(1) || z >= tentative.GetLength(2))
                        {
                            return false;
                        }

                        Brick b = tentative[x, y, z];
                        if (b == null)
                        {
                            tentative[x, y, z] = this.brick;                            
                        }
                        else
                        {
                            return false;
                        }
                        if (y > 0)
                        {
                            if (tentative[x, y - 1, z] != null)
                            {
                                lowerConnections++;
                            }
                        }
                        if (y < tentative.GetLength(1) - 1)
                        {
                            if (tentative[x, y + 1, z] != null)
                            {
                                upperConnections++;
                            }
                        }
                    }
                    
                }
            }
            if (apply == false)
            {
                if (lowerConnections > 0 && upperConnections > 0)
                {
                    return false;
                }
                if (lowerConnections == 0)
                {
                    if (upperConnections < 2)
                    {
                        return false;
                    }
                }
                if (upperConnections == 0)
                {
                    if (lowerConnections < 2)
                    {
                        return false;
                    }
                }
            }
            int count = 0;
            foreach (Point p in this.brick.points)
            //Parallel.ForEach(this.brick.points, p =>
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
                center += pCopy.position;
                count++;

                if (apply)
                {
                    pCopy.brickColor = this.color;
                    pCopy.brickColorInteger = (int)this.color;
                    pointGrid[x, y, z].Add(pCopy);
                }

            }//);
            if (apply){ 
                Array.Copy(tentative, voxelGrid, tentative.GetLength(0) * tentative.GetLength(1) * tentative.GetLength(2));
            }
            this.center /= count;
            return true;

        }

        
    }
}
