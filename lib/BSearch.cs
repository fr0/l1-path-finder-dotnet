using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public static class BSearch
  {
    static int SearchInternal<T,Y>(List<T> a, int l, int h, Y y, Func<T,Y,int> c)
    {
      var i = l - 1;
      while (l <= h)
      {
        var m = (int)((uint)(l + h) >> 1);
        var x = a[m];
        if (c(x, y) < 0)
        {
          i = m;
          l = m + 1;
        }
        else
        {
          h = m - 1;
        }
      }
      return i;
    }

    public static int Search<T,Y>(List<T> a, Y y, Func<T, Y, int> c)
    {
      return SearchInternal(a, 0, a.Count - 1, y, c);
    }
  }
}

