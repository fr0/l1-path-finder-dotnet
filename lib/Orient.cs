using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public class Orientation
  {
    const double EPSILON = 1.1102230246251565e-16;
    const double ERRBOUND3 = (3.0 + 16.0 * EPSILON) * EPSILON;

    static double Orientation3Exact(IPoint m0, IPoint m1, IPoint m2)
    {
      var p = ((m1.y * m2.x) + (-m2.y * m1.x)) + ((m0.y * m1.x) + (-m1.y * m0.x));
      var n = (m0.y * m2.x) + (-m2.y * m0.x);
      return p - n;
    }

    public static double Orient(IPoint a, IPoint b, IPoint c)
    {
      var l = (a.y - c.y) * (b.x - c.x);
      var r = (a.x - c.x) * (b.y - c.y);
      var det = l - r;
      double s = 0;
      if (l > 0)
      {
        if (r <= 0)
        {
          return det;
        }
        else
        {
          s = l + r;
        }
      }
      else if (l < 0)
      {
        if (r >= 0)
        {
          return det;
        }
        else
        {
          s = -(l + r);
        }
      }
      else
      {
        return det;
      }
      var tol = ERRBOUND3 * s;
      if (det >= tol || det <= -tol)
      {
        return det;
      }
      return Orientation3Exact(a, b, c);
    }
  }
}
