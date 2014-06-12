using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace XYZFileLoader
{

    public class Brick
    {
        public List<Point> points;
        public Color color;
        public Brick(Color color)
        {
            this.color = color;
            this.points = Reader.getPoints();
        }

    }
}
