using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KADA
{
    
    class ICPDataContainer
    {
        public DepthColor[,] dc;
        public Vector3 center;
        public List<Vector3> qi;

        public ICPDataContainer(DepthColor[,] dc, Vector3 center, List<Vector3> qi)
        {
            this.dc = dc;
            this.center = center;
            this.qi = qi;
        }
    }
}
