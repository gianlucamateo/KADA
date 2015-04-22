using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KADA
{
    public class TentativeModel : Model
    {


        public TentativeModel(List<LocatedBrick> bricks, LocatedBrick tentativeBrick, Vector3 center)
            : base(false, center, tentBrick: tentativeBrick, fast: true)
        {
            this.center = center;
            this.Bricks = new List<LocatedBrick>(bricks);
            this.Bricks.Add(tentativeBrick);
            this.TentativeBrick = tentativeBrick;
            //this.GenerateKDTree();
        }
        public TentativeModel(Model m)
            : base(false, m.center, tentBrick: null, fast: true)
        {
            this.center = m.center;
            this.Bricks = m.Bricks;
            this.TentativeBrick = null;
        }

        public Model validate(BrickColor color)
        {
            if (this.TentativeBrick != null)
                this.TentativeBrick.setColor(color);
            /*if (this.Bricks.Count > 3)
            {
                this.Bricks.Clear();
                this.Bricks.Add(TentativeBrick);
            }*/
            Model model = new Model(false, this.center, this.Bricks, this.TentativeBrick, fast: true);

            return model;
        }


    }
}
