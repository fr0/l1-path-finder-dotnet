using NumSharp;
using NumSharp.Generic;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public class Geometry
  {
    public List<IPoint> corners;
    public NDArray<int> grid;
    public Geometry(List<IPoint> corners, NDArray<int> grid)
    {
      this.corners = corners;
      this.grid = grid;
    }
    public bool StabRay(double vx, double vy, double x)
    {
      return this.StabBox(vx, vy, x, vy);
    }
    public bool StabTile(double x, double y)
    {
      return this.StabBox(x, y, x, y);
    }
    public double Integrate(double x, double y)
    {
      if (x < 0 || y < 0)
      {
        return 0;
      }
      return (double)this.grid[
        (int)Math.Min(x, this.grid.shape[0] - 1),
        (int)Math.Min(y, this.grid.shape[1] - 1)];
    }

    public bool StabBox(double ax, double ay, double bx, double by)
    {
      var lox = Math.Min(ax, bx);
      var loy = Math.Min(ay, by);
      var hix = Math.Max(ax, bx);
      var hiy = Math.Max(ay, by);


      var s = this.Integrate(lox - 1, loy - 1)
            - this.Integrate(lox - 1, hiy)
            - this.Integrate(hix, loy - 1)
            + this.Integrate(hix, hiy);

      return s > 0;
    }

    public static NDArray<int> CreateSummedAreaTable(NDArray<int> img)
    {
      var result = new NDArray<int>(img.shape);
      for (var x = 0; x < img.shape[0]; x++)
      {
        var sum = 0;
        for (var y = 0; y < img.shape[1]; y++)
        {
          sum += img[x, y];
          if (x == 0)
          {
            result[x, y] = sum;
          }
          else
          {
            result[x, y] = result[x - 1, y] + sum;
          }
        }
      }
      return result;
    }

    public static int ComparePoint(IPoint a, IPoint b)
    {
      var d = Math.Sign(a.x - b.x);
      if (d != 0) { return d; };
      return Math.Sign(a.y - b.y);
    }
    public static Geometry CreateGeometry(NDArray<int> grid)
    {
      var loops = Contour.GetContours(grid.transpose(), false);

      //Extract corners
      var corners = new List<IPoint>();
      for (var k = 0; k < loops.Count; ++k)
      {
        var polygon = loops[k];
        for (var i = 0; i < polygon.Count; ++i)
        {
          var a = polygon[(i + polygon.Count - 1) % polygon.Count];
          var b = polygon[i];
          var c = polygon[(i + 1) % polygon.Count];
          if (Orientation.Orient(a, b, c) > 0)
          {
            double x = 0, y = 0;
            if (b.x - a.x != 0)
            {
              x = b.x - a.x;
            }
            else
            {
              x = b.x - c.x;
            }
            x = b.x + Math.Min((int)Math.Round(x / Math.Abs(x)), 0);
            if (b.y - a.y != 0)
            {
              y = b.y - a.y;
            }
            else
            {
              y = b.y - c.y;
            }
            y = b.y + Math.Min((int)Math.Round(y / Math.Abs(y)), 0);
            var offset = new Vertex(x, y);

            if (offset.x >= 0 && offset.x < grid.shape[0] &&
                offset.y >= 0 && offset.y < grid.shape[1] &&
                grid[(int)offset.x, (int)offset.y] == 0)
            {
              corners.Add(offset);
            }
          }
        }
      }

      //Remove duplicate corners
      corners = Unique.Uniq(corners, ComparePoint, false);

      //Create integral image
      var img = new NDArray<int>(grid.shape);
      for (var x = 0; x < grid.shape[0]; x++)
      {
        for (var y = 0; y < grid.shape[1]; y++)
        {
          img[x, y] = (grid[x, y] > 0 ? 1 : 0);
        }
      }
      img = CreateSummedAreaTable(img);

      //Return resulting geometry
      return new Geometry(corners, img);
    }
  }
}




