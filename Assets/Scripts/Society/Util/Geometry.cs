﻿using UnityEngine;

class Geometry {

    //Calculates the minimum distance between two lines.
    public static float Distance(Vector4 a, Vector4 b) {
        return 0f;
    }

    public static Vector2 Center(Vector4 a) {
        return new Vector2(a.x+0.5f*(a.z-a.x),a.y+0.5f*(a.w-a.y));
    }

    public static float Radius(Vector2 pos, Vector4[] features) {
        float max = 0f;
        foreach(Vector4 feature in features) {
            float distance = MaxDistance(pos, feature);
            if (max < distance) max = distance;
        }
        return max;
    }

    public static float MaxDistance(Vector2 pos, Vector4 feature) {
        Vector2 vec = new Vector2(feature.x, feature.y);
        float result1 = (vec - pos).magnitude;
        vec.x = feature.z;
        vec.y = feature.w;
        float result2 = (vec - pos).magnitude;
        return result1 < result2 ? result2 : result1;
    }
}
