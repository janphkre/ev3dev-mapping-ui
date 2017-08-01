using System;
using UnityEngine;

class Geometry {

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
        return (float) Math.Sqrt(x*x+y*y);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Efficient Approximation of the Mahalanobis Distance for Tracking with the Kalman Filter *
     * See http://hdl.handle.net/10216/348                                                     *
     * Paper by R. R. Pinho, J. M. R. S Tavares and M. F. V. Correia                           *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    //Calculates the approximated mahalanobis distance between two points.
    public static float MahalanobisDistance(Vector2 a, Vector2 b, Matrix inversedCovariance) {
        var v = a - b;
        return ((v.x * v.x) / inversedCovariance[0, 0]) + ((v.y * v.y) / inversedCovariance[1, 1]);//TODO: which part of the (whole) inversed covariance matrix is to be used?
    }

    //Calculates the mahalanobis distance between two points.
    public static float RealMahalanobisDistance(Vector2 a, Vector2 b, Matrix inversedCovariance) {
        var v = a - b;
        return (float) Math.Sqrt((v * inversedCovariance * v)[0, 0]);
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
        var result = new Vector2(Math.Abs(f.magnitude), origin.z);
        result.y += (float)Math.Atan2(f.x, f.y);
        return result;
    }

    //Returns the range and bearing form the robot to the feature.
    //The y coordinate of feat will be ignored
    public static Vector2 ToRangeBearing(Vector3 feat, Vector3 origin) {
        var f = new Vector2(feat.x - origin.x, feat.z - origin.y);
        var result = new Vector2(Math.Abs(f.magnitude), origin.z);
        result.y += (float)Math.Atan2(f.x, f.y);
        return result;
    }

    //Returns the position of the feature from the range and bearing. (Converting polar coordinates into carthesic coordinates)
    public static Vector2 FromRangeBearing(float posRange, float posBearing) {
        return new Vector2((float) (posRange * Math.Cos(posBearing)), (float) (posRange * Math.Sin(posBearing)));
    }

    //Returns the position of the feature from the range and bearing and the origin pose.
    public static Vector2 FromRangeBearing(float posRange, float posBearing, Vector3 origin) {
        return new Vector2((float)(posRange * Math.Cos(posBearing + origin.z)) + origin.x, (float)(posRange * Math.Sin(posBearing + origin.z)) + origin.y);
    }

    //Checks wether the feature, provided as range and bearing, is within currently the drivable funnel of the 
    public static bool IsWithinFunnel(Vector2 featureRB) {
        if (featureRB.y == 0) return true;
        return Math.Sin(Math.Abs(featureRB.y)) * MainMenu.Physics.innerTurningDiameter + MainMenu.Physics.halfWheelbase < featureRB.x;
        //This is not completly correct. The turning circle should only be moved half wheelbase to the side!
    }
}
