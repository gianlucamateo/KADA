using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;
using System.Windows.Shapes;
using System.Drawing;
using System.Drawing.Imaging;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Controls;



namespace KADA
{
    class PointCloudViewer
    {
        private Viewport3D pcvFrame;
        MeshGeometry3D cubeMesh;
        private double cameraRotAngle = 0;

        private Transform3DGroup cameraTransform;
        private Transform3DCollection children = new Transform3DCollection();
        private ScaleTransform3D cameraScale = new ScaleTransform3D(1, 1, 1);
        private RotateTransform3D cameraRot = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0), new Point3D(0, 0, 0));
        private static ModelVisual3D[,] pixelGrid;

        public PointCloudViewer(Viewport3D pcvFrame)
        {
            pixelGrid = new ModelVisual3D[640, 480];

            this.pcvFrame = pcvFrame;
            cubeMesh= new MeshGeometry3D();
            cubeMesh.Positions.Add(new Point3D(0, 0, 0));
            cubeMesh.Positions.Add(new Point3D(0, 1, 0));
            cubeMesh.Positions.Add(new Point3D(1, 0, 0));

            cubeMesh.Positions.Add(new Point3D(0, 1, 0));
            cubeMesh.Positions.Add(new Point3D(1, 0, 0));
            cubeMesh.Positions.Add(new Point3D(1, 1, 0));

            cubeMesh.TriangleIndices.Add(0);
            cubeMesh.TriangleIndices.Add(2);
            cubeMesh.TriangleIndices.Add(1);

            cubeMesh.TriangleIndices.Add(3);
            cubeMesh.TriangleIndices.Add(4);
            cubeMesh.TriangleIndices.Add(5);

            Vector3D normal = new Vector3D(0, 1, 0);
            cubeMesh.Normals.Add(normal);
            cubeMesh.Normals.Add(normal);
            cubeMesh.Normals.Add(normal);

            cubeMesh.Normals.Add(normal);
            cubeMesh.Normals.Add(normal);
            cubeMesh.Normals.Add(normal);

            Material material = new DiffuseMaterial(
            new SolidColorBrush(Colors.DarkKhaki));
            GeometryModel3D triangleModel = new GeometryModel3D(
                cubeMesh, material);
            ModelVisual3D model = new ModelVisual3D();
            model.Content = triangleModel;

            ModelVisual3D PCRoot = new ModelVisual3D();
            this.pcvFrame.Children.Add(PCRoot);
/*
            for (int x = 0; x < 640; x++)
            {
                for (int y = 0; y < 480; y++)
                {
                    pixelGrid[x, y] = new ModelVisual3D();
                    pixelGrid[x, y].Content = triangleModel;//.Clone();
                    PCRoot.Children.Add(pixelGrid[x,y]);
                }
            }

*/            

            PCRoot.Children.Add(model);
            PCRoot.Transform = new ScaleTransform3D(new Vector3D(1, 1, 8), new Point3D(0, 0, 1));
            this.cameraTransform = new Transform3DGroup();
            this.children.Add(cameraScale);
            this.children.Add(cameraRot);
            this.cameraTransform.Children = this.children;
            this.pcvFrame.Camera.Transform = this.cameraTransform;
            


            
        }

        public void Rotate(double angle)
        {            
            this.cameraRotAngle += angle;
            this.cameraRot = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), cameraRotAngle), new Point3D(0, 0, 0));
            this.Update();
        }
        public void Update()
        {
            this.children.Clear();
            this.children.Add(cameraScale);
            this.children.Add(cameraRot);
        }
    }
}
