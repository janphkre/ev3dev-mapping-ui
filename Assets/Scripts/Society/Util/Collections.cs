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

public abstract class IFeature {
    public int index;
    protected RobotPose parent;

    public abstract bool IsFeature();
}

public class Feature: IFeature {
    public Vector4 feature;

    public Feature(Vector4 f, RobotPose p, int index) {
        feature = f;
        parent = p;
    }
    public override bool IsFeature() { return true; }
    public RobotPose ParentPose() { return parent; }
}

public class RobotPose: IFeature {
    public static RobotPose zero = new RobotPose(Vector3.zero,null, 0f);

    public Vector3 pose;
    public float radius;

    public RobotPose(Vector3 f, RobotPose p, float r) {
        pose = f;
        parent = p;
        radius = r;
    }
    public override bool IsFeature() { return false; }
    public RobotPose PreviousPose() { return parent; }
}