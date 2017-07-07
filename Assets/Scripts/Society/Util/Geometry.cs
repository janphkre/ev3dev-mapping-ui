using System;
using UnityEngine;

class Geometry {

    //Calculates the minimum distance between two lines.
    public static float Distance(Vector2 a, Vector2 b) {
        return Math.Abs((b - a).magnitude);
    }

    /*public static Vector2 Center(Vector4 a) {
        return new Vector2(a.x+0.5f*(a.z-a.x),a.y+0.5f*(a.w-a.y));
    }*/

    public static float Radius(Vector2 pos, Vector2[] features) {
        float max = 0f;
        foreach(Vector4 feature in features) {
            float distance = Distance(pos, feature);
            if (max < distance) max = distance;
        }
        return max;
    }

    //Feature is rotated around end by z (z should be degrees)
    public static Vector2 Rotate(Vector2 feature, Vector2 end, float z) {
        throw new NotImplementedException();
    }

    //Returns the range and bearing form the robot to the feature.
    public static Vector2 ToRangeBearing(Vector2 feat, Vector3 origin) {
        throw new NotImplementedException();
    }

    //Returns the position of the feature from the range and bearing. (Converting polar coordinates to carthesic coordinates)
    public static Vector2 FromRangeBearing(float posRange, float posBearing) {
        throw new NotImplementedException();
    }
}
