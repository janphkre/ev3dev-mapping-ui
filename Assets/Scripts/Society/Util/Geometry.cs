using System;
using System.Collections.Generic;
using UnityEngine;

class Geometry {

    //Calculates the distance between two points.
    public static float Distance(Vector2 a, Vector2 b) {
        return Math.Abs((b - a).magnitude);
    }

    //Calculates the mahalanobis distance between two points.
    public static float MahalanobisDistance(Vector2 a, Vector2 b) {
        throw new NotImplementedException();
    }

    //Calculates the maximum distance from a pose to all features:
    public static float Radius(Vector2 pos, Vector2[] features) {
        float max = 0f;
        foreach(Vector4 feature in features) {
            float distance = Distance(pos, feature);
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
        result.y += (float)(Math.Atan2(f.x, f.y) * 180.0d / Math.PI);
        result.y %= 360;
        return result;
    }

    //Returns the position of the feature from the range and bearing. (Converting polar coordinates(with degrees) to carthesic coordinates)
    public static Vector2 FromRangeBearing(float posRange, float posBearing) {
        double d = posBearing * Math.PI / 180.0d;
        return new Vector2((float) (posRange * Math.Cos(d)), (float) (posRange * Math.Sin(d)));
    }

    //Returns the position of the feature from the range and bearing and the origin pose.
    public static Vector2 FromRangeBearing(float posRange, float posBearing, Vector3 origin) {
        double d = (posBearing+origin.z) * Math.PI / 180.0d;
        return new Vector2((float)(posRange * Math.Cos(d)) + origin.x, (float)(posRange * Math.Sin(d)) + origin.y);
    }
}
