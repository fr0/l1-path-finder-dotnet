using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public static class Unique
  {
    static List<T> Unique_pred<T>(List<T> list, Comparison<T> compare)
    {
      var ptr = 1;
      var len = list.Count;
      var a = list[0];
      var b = list[0];
      for (var i = 1; i < len; ++i)
      {
        b = a;
        a = list[i];
        if (compare(a, b) != 0)
        {
          if (i == ptr)
          {
            ptr++;
            continue;
          }
          list[ptr++] = a;
        }
      }
      return list.Take(ptr).ToList();
    }

    public static List<T> Uniq<T>(List<T> list, Comparison<T> compare, bool sorted)
    {
      if (list.Count == 0)
      {
        return list;
      }
      if (!sorted)
      {
        list = new List<T>(list);
        list.Sort(compare);
      }
      return Unique_pred(list, compare);
    }
  }
}
