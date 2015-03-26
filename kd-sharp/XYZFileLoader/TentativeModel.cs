using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KADA
{
    public class TentativeModel : Model
    {
        public LocatedBrick TentativeBrick;

        public TentativeModel(List<LocatedBrick> bricks, LocatedBrick tentativeBrick) : base(false,fast:false)
        {
            this.Bricks = new List<LocatedBrick>(bricks);
            this.Bricks.Add(tentativeBrick);
            this.TentativeBrick = tentativeBrick;
            //this.GenerateKDTree();
        }
        //new code, could be buggy
        public Model validate()
        {
            Model model = new Model(true,this.Bricks,fast: true);
            //model.Bricks = this.Bricks;
            //model.GenerateKDTree();
            return model;
        }
        

    }
}
