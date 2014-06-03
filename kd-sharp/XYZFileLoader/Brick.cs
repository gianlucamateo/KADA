using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XYZFileLoader
{

    class Brick
    {
        public List<Point> points;
        public Brick()
        {
            this.points = Reader.getPoints();
        }

    }
}
