using NumSharp;
using NumSharp.Core;
using NumSharp.Generic;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public class Segment
  {
    public double start;
    public double end;
    public bool direction;
    public double height;
    public bool visited;
    public Segment next;
    public Segment prev;

    public Segment(double start, double end, bool direction, double height)
    {
      this.start = start;
      this.end = end;
      this.direction = direction;
      this.height = height;
      this.visited = false;
      this.next = null;
      this.prev = null;
    }
  }
  public class ContourVertex : IPoint
  {
    public double x;
    public double y;
    public Segment segment;
    public int orientation;
    public ContourVertex(double x, double y, Segment segment, int orientation)
    {
      this.x = x;
      this.y = y;
      this.segment = segment;
      this.orientation = orientation;
    }

    double IPoint.x => this.x;
    double IPoint.y => this.y;
  }

  public class Contour
  {
    static List<Segment> GetParallelCountours(NDArray array, bool direction)
    {
      var n = array.shape[0];
      var m = array.shape[1];
      var contours = new List<Segment>();
      //Scan top row
      var a = false;
      var b = false;
      var c = false;
      var d = false;
      var x0 = 0;
      var i = 0;
      var j = 0;
      for (j = 0; j < m; ++j)
      {
        b = ((int)array[0, j] != 0);
        if (b == a)
        {
          continue;
        }
        if (a)
        {
          contours.Add(new Segment(x0, j, direction, 0));
        }
        if (b)
        {
          x0 = j;
        }
        a = b;
      }
      if (a)
      {
        contours.Add(new Segment(x0, j, direction, 0));
      }
      //Scan center
      for (i = 1; i < n; ++i)
      {
        a = false;
        b = false;
        x0 = 0;
        for (j = 0; j < m; ++j)
        {
          c = (int)array[i - 1, j] != 0;
          d = (int)array[i, j] != 0;
          if (c == a && d == b)
          {
            continue;
          }
          if (a != b)
          {
            if (a)
            {
              contours.Add(new Segment(j, x0, direction, i));
            }
            else
            {
              contours.Add(new Segment(x0, j, direction, i));
            }
          }
          if (c != d)
          {
            x0 = j;
          }
          a = c;
          b = d;
        }
        if (a != b)
        {
          if (a)
          {
            contours.Add(new Segment(j, x0, direction, i));
          }
          else
          {
            contours.Add(new Segment(x0, j, direction, i));
          }
        }
      }
      //Scan bottom row
      a = false;
      x0 = 0;
      for (j = 0; j < m; ++j)
      {
        b = (int)array[n - 1, j] != 0;
        if (b == a)
        {
          continue;
        }
        if (a)
        {
          contours.Add(new Segment(j, x0, direction, n));
        }
        if (b)
        {
          x0 = j;
        }
        a = b;
      }
      if (a)
      {
        contours.Add(new Segment(j, x0, direction, n));
      }
      return contours;
    }

    static List<ContourVertex> GetVertices(List<Segment> contours)
    {
      var vertices = new List<ContourVertex>(new ContourVertex[contours.Count * 2]);
      for (var i = 0; i < contours.Count; ++i)
      {
        var h = contours[i];
        if (!h.direction)
        {
          vertices[2 * i] = new ContourVertex(h.start, h.height, h, 0);
          vertices[2 * i + 1] = new ContourVertex(h.end, h.height, h, 1);
        }
        else
        {
          vertices[2 * i] = new ContourVertex(h.height, h.start, h, 0);
          vertices[2 * i + 1] = new ContourVertex(h.height, h.end, h, 1);
        }
      }
      return vertices;
    }

    public static List<Point> Walk(Segment v, bool clockwise)
    {
      var result = new List<Point>();
      while (!v.visited)
      {
        v.visited = true;
        if (v.direction)
        {
          result.Add(new Point(v.height, v.end));
        }
        else
        {
          result.Add(new Point(v.start, v.height));
        }
        if (clockwise)
        {
          v = v.next;
        }
        else
        {
          v = v.prev;
        }
      }
      return result;
    }

    static int CompareVertex(ContourVertex a, ContourVertex b)
    {
      var d = a.x - b.x;
      if (d != 0)
      {
        return Math.Sign(d);
      }
      d = a.y - b.y;
      if (d != 0)
      {
        return Math.Sign(d);
      }
      return a.orientation - b.orientation;
    }

    public static List<List<Point>> GetContours(NDArray array, bool clockwise)
    {
      //First extract horizontal contours and vertices
      var hcontours = GetParallelCountours(array, false);
      var hvertices = GetVertices(hcontours);
      hvertices.Sort(CompareVertex);

      //Extract vertical contours and vertices
      var vcontours = GetParallelCountours(array.transpose(), true);
      var vvertices = GetVertices(vcontours);
      vvertices.Sort(CompareVertex);

      //Glue horizontal and vertical vertices together
      var nv = hvertices.Count;
      for (var i = 0; i < nv; ++i)
      {
        var h = hvertices[i];
        var v = vvertices[i];
        if (h.orientation != 0)
        {
          h.segment.next = v.segment;
          v.segment.prev = h.segment;
        }
        else
        {
          h.segment.prev = v.segment;
          v.segment.next = h.segment;
        }
      }

      //Unwrap loops
      var loops = new List<List<Point>>();
      for (var i = 0; i < hcontours.Count; ++i)
      {
        var h = hcontours[i];
        if (!h.visited)
        {
          loops.Add(Walk(h, clockwise));
        }
      }

      //Return
      return loops;
    }
  }
}
