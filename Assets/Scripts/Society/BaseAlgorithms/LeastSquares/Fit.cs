﻿
namespace ev3dev.Society.LeastSquares
{
  using UnityEngine;
  using System.Collections.Generic;
  using System.Linq;

  public abstract class Fit
  {
    public Fit(IEnumerable<Vector3> points) {
        Points = points;
        var uniquePoints = new List<Vector2>();
        UniquePoints = uniquePoints;
        var grp = Points.GroupBy((p) => { return p.x; });
        foreach (IGrouping<float, Vector3> g in grp) uniquePoints.Add(new Vector2(g.Key, g.Select(p => p.z).Average()));
    }

    protected IEnumerable<Vector3> Points;
    /// <summary>
    /// original points count
    /// </summary>
    public int Count { get { return Points.Count(); } }

    /// <summary>
    /// group points with equal x value, average group y value
    /// </summary>
    protected IEnumerable<Vector2> UniquePoints;
    /// <summary>
    /// count of point set used for interpolation
    /// </summary>
    public int CountUnique { get { return UniquePoints.Count(); } }
    /// <summary>
    /// abscissae
    /// </summary>
    public IEnumerable<float> X { get { return UniquePoints.Select(p => p.x); } }
    public IEnumerable<float> X_Orig { get { return Points.Select(p => p.x); } }
    /// <summary>
    /// ordinates
    /// </summary>
    public IEnumerable<float> Y { get { return UniquePoints.Select(p => p.y); } }
    public IEnumerable<float> Y_Orig { get { return Points.Select(p => p.z); } }

    /// <summary>
    /// x mean
    /// </summary>
    protected float AverageX { get { return X.Average(); } }
    /// <summary>
    /// y mean
    /// </summary>
    protected float AverageY { get { return Y.Average(); } }

    /// <summary>
    /// x sum
    /// </summary>
    protected float SumX { get { return X.Sum(); } }
    /// <summary>
    /// y sum
    /// </summary>
    protected float SumY { get { return Y.Sum(); } }

    /// <summary>
    /// total sum of squares (proportional to the sample variance);
    /// sum sq (yi - mean y)
    /// </summary>
    public float SSTotal { get { var meanY = Y.Select(y => y - AverageY); return meanY.DotProduct(meanY); } }
  }
}
