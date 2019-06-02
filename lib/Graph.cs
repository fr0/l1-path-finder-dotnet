using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public class Graph
  {
    readonly Vertex NIL = Vertex.NIL;
    const int NUM_LANDMARKS = Vertex.NUM_LANDMARKS;

    public Vertex target;
    public List<Vertex> verts = new List<Vertex>();
    public Vertex freeList;
    public Vertex toVisit;
    public Vertex lastS;
    public Vertex lastT;
    public double srcX = 0;
    public double srcY = 0;
    public double dstX = 0;
    public double dstY = 0;
    public List<object> landmarks = new List<object>();
    public List<double> landmarkDist;


    public static double heuristic(List<double> tdist, double tx, double ty, Vertex node)
    {
      var nx = +node.x;
      var ny = +node.y;
      var pi = Math.Abs(nx - tx) + Math.Abs(ny - ty);
      var ndist = node.landmark;
      for (var i = 0; i < NUM_LANDMARKS; ++i)
      {
        pi = Math.Max(pi, tdist[i] - ndist[i]);
      }
      return 1.0000009536743164 * pi;
    }

    public Graph()
    {
      this.target = Vertex.CreateVertex(0, 0);
      this.freeList = this.target;
      this.toVisit = NIL;
      this.lastS = null;
      this.lastT = null;
      this.landmarkDist = Vertex.LANDMARK_DIST();
    }

    public Vertex AddVertex(double x, double y)
    {
      var v = Vertex.CreateVertex(x, y);
      this.verts.Add(v);
      return v;
    }

    public void Link(Vertex u, Vertex v)
    {
      Vertex.Link(u, v);
    }

    public void setSourceAndTarget(double sx, double sy, double tx, double ty)
    {
      this.srcX = sx;
      this.srcY = sy;
      this.dstX = tx;
      this.dstY = ty;
    }

    //Mark vertex connected to source
    public void AddS(Vertex v)
    {
      if ((v.state & 2) == 0)
      {
        v.heuristic = heuristic(this.landmarkDist, this.dstX, this.dstY, v);
        v.weight = Math.Abs(this.srcX - v.x) + Math.Abs(this.srcY - v.y) + v.heuristic;
        v.state |= 2;
        v.pred = null;
        this.toVisit = Vertex.Push(this.toVisit, v);
        this.freeList = Vertex.Insert(this.freeList, v);
        this.lastS = v;
      }
    }

    //Mark vertex connected to target
    public void AddT(Vertex v)
    {
      if ((v.state & 1) == 0)
      {
        v.state |= 1;
        this.freeList = Vertex.Insert(this.freeList, v);
        this.lastT = v;

        //Update heuristic
        var d = Math.Abs(v.x - this.dstX) + Math.Abs(v.y - this.dstY);
        var vdist = v.landmark;
        var tdist = this.landmarkDist;
        for (var i = 0; i < NUM_LANDMARKS; ++i)
        {
          tdist[i] = Math.Min(tdist[i], vdist[i] + d);
        }
      }
    }

    //Retrieves the path from dst->src
    public List<Point> GetPath(List<Point> outpath = null)
    {
      if (outpath == null)
      {
        outpath = new List<Point>();
      }
      var prevX = this.dstX;
      var prevY = this.dstY;
      outpath.Add(new Point(prevX, prevY));
      var head = this.target.pred;
      while (head != null)
      {
        if (prevX != head.x && prevY != head.y)
        {
          outpath.Add(new Point(head.x, prevY));
        }
        if (prevX != head.x || prevY != head.y)
        {
          outpath.Add(new Point(head.x, head.y));
        }
        prevX = head.x;
        prevY = head.y;
        head = head.pred;
      }
      if (prevX != this.srcX && prevY != this.srcY)
      {
        outpath.Add(new Point(this.srcX, prevY));
      }
      if (prevX != this.srcX || prevY != this.srcY)
      {
        outpath.Add(new Point(this.srcX, this.srcY));
      }
      return outpath;
    }

    public List<List<Vertex>> FindComponents()
    {
      var verts = this.verts;
      var n = verts.Count;
      for (var i = 0; i < n; ++i)
      {
        verts[i].component = -1;
      }
      var components = new List<List<Vertex>>();
      for (var i = 0; i < n; ++i)
      {
        var root = verts[i];
        if (root.component >= 0)
        {
          continue;
        }
        var label = components.Count;
        root.component = label;
        var toVisit = new List<Vertex>(new[] { root });
        var ptr = 0;
        while (ptr < toVisit.Count)
        {
          var v = toVisit[ptr++];
          var adj = v.edges;
          for (var j = 0; j < adj.Count; ++j)
          {
            var u = adj[j];
            if (u.component >= 0)
            {
              continue;
            }
            u.component = label;
            toVisit.Add(u);
          }
        }
        components.Add(toVisit);
      }
      return components;
    }

    //Find all landmarks
    public static int CompareVert(IPoint a, IPoint b)
    {
      var d = Math.Sign(a.x - b.x);
      if (d != 0) { return d; }
      return Math.Sign(a.y - b.y);
    }

    //For each connected component compute a set of landmarks
    public void FindLandmarks(List<Vertex> component)
    {
      component.Sort(CompareVert);
      var v = component[(int)((uint)component.Count >> 1)];
      for (var k = 0; k < NUM_LANDMARKS; ++k)
      {
        v.weight = 0.0;
        this.landmarks.Add(v);
        for (var toVisit = v; toVisit != NIL;)
        {
          v = toVisit;
          v.state = 2;
          toVisit = Vertex.Pop(toVisit);
          var w = v.weight;
          var adj = v.edges;
          for (var i = 0; i < adj.Count; ++i)
          {
            var u = adj[i];
            if (u.state == 2)
            {
              continue;
            }
            var d = w + Math.Abs(v.x - u.x) + Math.Abs(v.y - u.y);
            if (u.state == 0)
            {
              u.state = 1;
              u.weight = d;
              toVisit = Vertex.Push(toVisit, u);
            }
            else if (d < u.weight)
            {
              u.weight = d;
              toVisit = Vertex.DecreaseKey(toVisit, u);
            }
          }
        }
        var farthestD = 0.0;
        for (var i = 0; i < component.Count; ++i)
        {
          var u = component[i];
          u.state = 0;
          u.landmark[k] = u.weight;
          var s = double.PositiveInfinity;
          for (var j = 0; j <= k; ++j)
          {
            s = Math.Min(s, u.landmark[j]);
          }
          if (s > farthestD)
          {
            v = u;
            farthestD = s;
          }
        }
      }
    }

    public void Init()
    {
      var components = this.FindComponents();
      for (var i = 0; i < components.Count; ++i)
      {
        this.FindLandmarks(components[i]);
      }
    }

    //Runs a* on the graph
    public double Search()
    {
      var target = this.target;
      var freeList = this.freeList;
      var tdist = this.landmarkDist;

      //Initialize target properties
      var dist = double.PositiveInfinity;

      //Test for case where S and T are disconnected
      if (this.lastS != null && this.lastT != null &&
          this.lastS.component == this.lastT.component)
      {

        var sx = +this.srcX;
        var sy = +this.srcY;
        var tx = +this.dstX;
        var ty = +this.dstY;

        for (var toVisit = this.toVisit; toVisit != NIL;)
        {
          var node = toVisit;
          var nx = +node.x;
          var ny = +node.y;
          var d = Math.Floor(node.weight - node.heuristic);

          if (node.state == 3)
          {
            //If node is connected to target, exit
            dist = d + Math.Abs(tx - nx) + Math.Abs(ty - ny);
            target.pred = node;
            break;
          }

          //Mark node closed
          node.state = 4;

          //Pop node from toVisit queue
          toVisit = Vertex.Pop(toVisit);

          var adj = node.edges;
          var n = adj.Count;
          for (var i = 0; i < n; ++i)
          {
            var v = adj[i];
            var state = v.state;
            if (state == 4)
            {
              continue;
            }
            var vd = d + Math.Abs(nx - v.x) + Math.Abs(ny - v.y);
            if (state < 2)
            {
              var vh = heuristic(tdist, tx, ty, v);
              v.state |= 2;
              v.heuristic = vh;
              v.weight = vh + vd;
              v.pred = node;
              toVisit = Vertex.Push(toVisit, v);
              freeList = Vertex.Insert(freeList, v);
            }
            else
            {
              var vw = vd + v.heuristic;
              if (vw < v.weight)
              {
                v.weight = vw;
                v.pred = node;
                toVisit = Vertex.DecreaseKey(toVisit, v);
              }
            }
          }
        }
      }

      //Clear the free list & priority queue
      Vertex.Clear(freeList);

      //Reset pointers
      this.freeList = target;
      this.toVisit = NIL;
      this.lastS = this.lastT = null;

      //Reset landmark distance
      for (var i = 0; i < NUM_LANDMARKS; ++i)
      {
        tdist[i] = double.PositiveInfinity;
      }

      //Return target distance
      return dist;
    }
  }
}
