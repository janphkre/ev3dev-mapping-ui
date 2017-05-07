using UnityEngine;

class Geometry {

    //Calculates the minimum distance between two lines.
    public static float Distance(Vector4 a, Vector4 b) {
        return 0f;
    }

    public static Vector2 Center(Vector4 a) {
        return new Vector2(a.x+0.5f*(a.z-a.x),a.y+0.5f*(a.w-a.y));
    }
}
