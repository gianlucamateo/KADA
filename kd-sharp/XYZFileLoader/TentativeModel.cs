using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KADA
{
    public class TentativeModel : Model
    {
        public LocatedBrick TentativeBrick;

        public TentativeModel(List<LocatedBrick> bricks, LocatedBrick tentativeBrick, Vector3 center) : base(false,center,fast:true)
        {
            this.center = center;
            this.Bricks = new List<LocatedBrick>(bricks);
            this.Bricks.Add(tentativeBrick);
            this.TentativeBrick = tentativeBrick;
            //this.GenerateKDTree();
        }
        //new code, could be buggy
        public Model validate(BrickColor color)
        {
            this.TentativeBrick.setColor(color);
            Model model = new Model(true, this.center, this.Bricks, fast: true);
            //model.Bricks = this.Bricks;
            //model.GenerateKDTree();
            return model;
        }
        

    }
}
