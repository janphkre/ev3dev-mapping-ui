using System;
using System.Collections.Generic;
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

    public static Vector2 Rotate(Vector2 feature, Vector2 end, float z) {
        throw new NotImplementedException();
    }
}
