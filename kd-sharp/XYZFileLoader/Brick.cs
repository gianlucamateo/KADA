using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XYZFileLoader
{

    public class Brick
    {
        public List<Point> points;
        public Brick()
        {
            this.points = Reader.getPoints();
        }

    }
}
