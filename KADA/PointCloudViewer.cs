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
        private double cameraRot = 0;

        public PointCloudViewer(Viewport3D pcvFrame)
        {
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
            this.pcvFrame.Children.Add(model);
        }

        public void Rotate(double angle)
        {
            cameraRot += angle;
            this.pcvFrame.Camera.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), cameraRot), new Point3D(0, 0, 0));
        }
    }
}
