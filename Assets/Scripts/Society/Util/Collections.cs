using UnityEngine;

public class FeatureCollection {
    //The map is formed out of features. Each feature is a line. x,y form the start of the line, z,w the end of the line. x <= z should be valid.
    public Vector4[] map;
    public Vector3 end;
    public float radius;

    public FeatureCollection(int size) {
        map = new Vector4[size];
    }

    public FeatureCollection(FeatureCollection otherMap) {
        map = new Vector4[otherMap.map.Length];
        otherMap.map.CopyTo(map, 0);
    }
}

public class ObservedFeature {

    public Vector4 feature;
    public int observedCount;

    public ObservedFeature(Vector4 feature) {
        this.feature = feature;
        observedCount = 1;
    }
}