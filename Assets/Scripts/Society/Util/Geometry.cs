
using UnityEngine;

namespace ev3devMapping.Society {

public class Geometry {

    public const float HALF_CIRCLE = Mathf.PI;
    public const float RIGHT_ANGLE = HALF_CIRCLE / 2f;
    public const float FULL_CIRCLE = HALF_CIRCLE * 2f;
    public const float EIGHTH_CIRCLE = RIGHT_ANGLE / 2f;
    public const float FLOAT_PRECISION  = 0.0001f;

    //Calculates the distance between two points.
    public static float EuclideanDistance(Vector2 a, Vector2 b) {
        return (b - a).magnitude;
    }

    //Calculates the distance between two points.
    public static float EuclideanDistance(Vector3 a, Vector3 b) {
        return (b - a).magnitude;
    }

    //Calculates the distance between two points. a.y is ignored.
    public static float EuclideanDistance(Vector3 a, Vector2 b) {
        var x = b.x - a.x;
        var y = b.y - a.z;
        return Mathf.Sqrt(x*x+y*y);
    }

    public static float EuclideanDistance(GraphNode a, GraphNode b) {
        return (b.Position - a.Position).magnitude;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Efficient Approximation of the Mahalanobis Distance for Tracking with the Kalman Filter *
     * See http://hdl.handle.net/10216/348                                                     *
     * Paper by R. R. Pinho, J. M. R. S Tavares and M. F. V. Correia                           *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    //Calculates the non-rooted mahalanobis distance between two points.
    public static float SquaredMahalanobisDistance(Vector2 a, Vector2 b, Matrix inversedCovariance) {
        var v = a - b;
        return v.x * (v.y * (inversedCovariance[1, 0] + inversedCovariance[0, 1]) + inversedCovariance[0, 0] * v.x ) + inversedCovariance[1, 1] * v.y * v.y;
    }

    //Calculates the mahalanobis distance between two points.
    public static float RealMahalanobisDistance(Vector2 a, Vector2 b, Matrix inversedCovariance) {
        Vector2 v = a - b;
        return Mathf.Sqrt((v * inversedCovariance * v)[0, 0]);
    }

    //Calculates the maximum distance from a pose to all features:
    public static float Radius(Vector2 pos, Vector2[] features) {
        float max = 0f;
        foreach(Vector4 feature in features) {
            float distance = EuclideanDistance(pos, feature);
            if (max < distance) max = distance;
        }
        return max;
    }

    //Feature is rotated around end by z (in degrees)
    public static Vector2 Rotate(Vector2 feature, Vector3 end, float z) {
        var f = ToRangeBearing(feature, end);
        return FromRangeBearing(f.x, f.y + z, end);
    }

    //Returns the range and bearing form the robot to the feature.
    public static Vector2 ToRangeBearing(Vector2 feat, Vector3 origin) {
        var f = feat - (Vector2) origin;
        var result = new Vector2(Mathf.Abs(f.magnitude), -origin.z);
        result.y += Mathf.Atan2(f.x, f.y);
        return result;
    }

    //Returns the range and bearing form the robot to the feature.
    //The y coordinate of feat will be ignored.
    public static Vector2 ToRangeBearing(Vector3 feat, Vector3 origin) {
        var f = new Vector2(feat.x - origin.x, feat.z - origin.y);
        var result = new Vector2(Mathf.Abs(f.magnitude), -origin.z);
        result.y += Mathf.Atan2(f.x, f.y);
        return result;
    }

    //Returns the position of the feature from the range and bearing. (Converting polar coordinates into carthesic coordinates)
    public static Vector2 FromRangeBearing(float posRange, float posBearing) {
        return new Vector2((posRange * Mathf.Cos(posBearing)), (posRange * Mathf.Sin(posBearing)));
    }

    //Returns the position of the feature from the range and bearing and the origin pose.
    public static Vector2 FromRangeBearing(float posRange, float posBearing, Vector3 origin) {
        return new Vector2((posRange * Mathf.Cos(posBearing + origin.z)) + origin.x, (posRange * Mathf.Sin(posBearing + origin.z)) + origin.y);
    }

    //Checks wether the feature, provided as range and bearing, is within currently the drivable funnel of the 
    public static bool IsWithinFunnel(Vector2 featureRB) {
        if (featureRB.y == 0) return true;
        return Mathf.Sin(Mathf.Abs(featureRB.y)) * MainMenu.Physics.innerTurningDiameter - MainMenu.Physics.halfWheelbase < featureRB.x;
        //This is not completly correct. The turning circle should only be moved half wheelbase to the side!
    }

    //Mathematical Modulo (Non-Symmetrical)
    public static int Modulo(int x, int y) {
        int r = x % y;
        return r < 0 ? r + y : r;
    }
    
    //Compares two floats with a reduced precision. This is used in the Matrix.Equals() methods.
    public static bool CompareFloats(float a, float b) {
        return Mathf.Abs(a-b) < FLOAT_PRECISION;
    }
}
}