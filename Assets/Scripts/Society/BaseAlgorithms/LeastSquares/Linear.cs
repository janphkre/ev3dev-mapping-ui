namespace LeastSquares
{
    using UnityEngine;
    using System.Collections.Generic;
    using System.Linq;

    /// <summary>
    /// Linear Interpolation using the least squares method
    /// <remarks>http://mathworld.wolfram.com/LeastSquaresFitting.html</remarks> 
    /// </summary>
    public class Linear : Fit
    {

        public Linear(IEnumerable<Vector3> points) : base(points) { }
        /// <summary>
        /// the computed slope, aka regression coefficient
        /// </summary>
        public Vector3 Slope { get { return new Vector3(ssxy,ssxx).normalized; } }

        // dotvector(x,y)-n*avgx*avgy
        float ssxy { get { return X.DotProduct(Y) - CountUnique * AverageX * AverageY; } }
        //sum squares x - n * square avgx
        float ssxx { get { return X.DotProduct(X) - CountUnique * AverageX * AverageX; } }

        /// <summary>
        /// computed  intercept
        /// </summary>
        public Vector3 Average { get { return new Vector3(AverageY, AverageX); } }

        public Ray Ray { get { return new Ray(Average, Slope); } }
    }
}









