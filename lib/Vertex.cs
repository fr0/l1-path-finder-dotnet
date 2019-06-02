using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace L1PathFinder
{
  public interface IPoint
  {
    double x { get; }
    double y { get; }
  }

  //Vertices have to do multiple things
  //
  //  1.  They store the topology of the graph which is gonna get searched
  //  2.  They implement the pairing heap data sturcture (intrusively)
  //  3.  They implement a linked list for tracking clean up
  //  4.  Track search information (keep track of predecessors, distances, open state)
  //
  public class Vertex : IPoint
  {
    public readonly double x;
    public readonly double y;
    public double heuristic = 0.25;
    public double weight = 0.25;
    public Vertex left = null;
    public Vertex right = null;
    public Vertex parent = null;
    public int state = 0;
    public Vertex pred = null;
    public Vertex nextFree = null;
    public List<Vertex> edges = new List<Vertex>();
    public List<double> landmark;
    public int component = 0;

    public static readonly Vertex NIL;

    static Vertex()
    {
      //Sentinel node
      NIL = new Vertex(double.PositiveInfinity, double.PositiveInfinity);
      NIL.weight = double.NegativeInfinity;
      NIL.left = NIL.right = NIL.parent = NIL;
    }

    public Vertex(double x, double y)
    {
      this.x = x;
      this.y = y;
      this.landmark = LANDMARK_DIST();
    }

    public const int NUM_LANDMARKS = 16;

    double IPoint.x => this.x;
    double IPoint.y => this.y;

    public override bool Equals(object o)
    {
      if (o is IPoint)
      {
        IPoint c = (IPoint)o;
        return (int)(c.x * 100) == (int)(this.x * 100) &&
               (int)(c.y * 100) == (int)(this.y * 100);
      }
      return false;
    }
    public override int GetHashCode()
    {
      return (((int)x & 0xFFFF) << 16) + ((int)y & 0xFFFF);
    }

    public static List<double> LANDMARK_DIST()
    {
      var res = new List<double>();
      for (var count = 0; count < NUM_LANDMARKS; ++count) {
        res.Add(double.PositiveInfinity);
      }
      return res;
    }

    //Heap insertion
    static Vertex HeapInsert(Vertex a, Vertex b)
    {
      var al = a.left;
      b.right = al;
      al.parent = b;
      b.parent = a;
      a.left = b;
      a.right = NIL;
      return a;
    }

    static Vertex Merge(Vertex a, Vertex b)
    {
      if (a == NIL)
      {
        return b;
      }
      else if (b == NIL)
      {
        return a;
      }
      else if (a.weight < b.weight)
      {
        return HeapInsert(a, b);
      }
      else
      {
        return HeapInsert(b, a);
      }
    }

    public static Vertex Push(Vertex root, Vertex node)
    {
      if (root == NIL)
      {
        return node;
      }
      else if (root.weight < node.weight)
      {
        var l = root.left;
        node.right = l;
        l.parent = node;
        node.parent = root;
        root.left = node;
        return root;
      }
      else
      {
        var l = node.left;
        root.right = l;
        l.parent = root;
        root.parent = node;
        node.left = root;
        return node;
      }
    }

    public static Vertex Pop(Vertex root)
    {
      var p = root.left;
      root.left = NIL;
      root = p;
      while (true)
      {
        var q = root.right;
        if (q == NIL)
        {
          break;
        }
        p = root;
        var r = q.right;
        var s = Merge(p, q);
        root = s;
        while (true)
        {
          p = r;
          q = r.right;
          if (q == NIL)
          {
            break;
          }
          r = q.right;
          s = s.right = Merge(p, q);
        }
        s.right = NIL;
        if (p != NIL)
        {
          p.right = root;
          root = p;
        }
      }
      root.parent = NIL;
      return root;
    }

    public static Vertex DecreaseKey(Vertex root, Vertex p)
    {
      var q = p.parent;
      if (q.weight < p.weight)
      {
        return root;
      }
      var r = p.right;
      r.parent = q;
      if (q.left == p)
      {
        q.left = r;
      }
      else
      {
        q.right = r;
      }
      if (root.weight <= p.weight)
      {
        var l = root.left;
        l.parent = p;
        p.right = l;
        root.left = p;
        p.parent = root;
        return root;
      }
      else
      {
        var l = p.left;
        root.right = l;
        l.parent = root;
        p.left = root;
        root.parent = p;
        p.right = p.parent = NIL;
        return p;
      }
    }

    //Topology
    public static Vertex CreateVertex(double x, double y)
    {
      var result = new Vertex(x, y);
      result.left = result.right = result.parent = NIL;
      return result;
    }

    public static void Link(Vertex u, Vertex v)
    {
      u.edges.Add(v);
      v.edges.Add(u);
    }

    //Free list functions
    public static Vertex Insert(Vertex list, Vertex node)
    {
      if (node.nextFree != null)
      {
        return list;
      }
      node.nextFree = list;
      return node;
    }

    public static void Clear(Vertex v)
    {
      while (v != null)
      {
        var next = v.nextFree;
        v.state = 0;
        v.left = v.right = v.parent = NIL;
        v.nextFree = null;
        v = next;
      }
    }
  }
}

