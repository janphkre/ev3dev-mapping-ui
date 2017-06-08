using System;
using System.Collections.Generic;
using UnityEngine;

class Geometry {

    //Calculates the minimum distance between two lines.
    public static float Distance(Vector4 a, Vector4 b) {
        throw new NotImplementedException();
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

    public static Vector4 Rotate(Vector4 vector4, Vector2 end, float z) {
        throw new NotImplementedException();
    }

    public static Vector4 GetFeature(List<FeatureCollection> list, int index) {
        throw new NotImplementedException();
    }

    //Returns true if the found element is a feature
    //Returns false if the found element is a robot pose
    internal static bool GetFeature(List<FeatureCollection> list, int index, out object obj) {
        throw new NotImplementedException();
    }
}
