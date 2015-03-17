using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using Color = System.Drawing.Color;
using Microsoft.Xna.Framework;

namespace KADA
{
    interface IHistogram
    {
        KADA.BrickColor GetColor();
        int GetValue(Vector3 color);
    }
}
