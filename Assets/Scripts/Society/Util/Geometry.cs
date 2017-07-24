﻿using System;
using UnityEngine;

class Geometry {

    //Calculates the distance between two points.
    public static float EuclideanDistance(Vector2 a, Vector2 b) {
        return Math.Abs((b - a).magnitude);
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

    //Returns the position of the feature from the range and bearing. (Converting polar coordinates into carthesic coordinates)
    public static Vector2 FromRangeBearing(float posRange, float posBearing) {
        return new Vector2((float) (posRange * Math.Cos(posBearing)), (float) (posRange * Math.Sin(posBearing)));
    }

    //Returns the position of the feature from the range and bearing and the origin pose.
    public static Vector2 FromRangeBearing(float posRange, float posBearing, Vector3 origin) {
        return new Vector2((float)(posRange * Math.Cos(posBearing + origin.z)) + origin.x, (float)(posRange * Math.Sin(posBearing + origin.z)) + origin.y);
    }
}
