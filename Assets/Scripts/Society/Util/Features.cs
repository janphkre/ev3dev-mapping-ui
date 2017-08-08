using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObservedFeature {

    public Vector2 feature;
    public int observedCount;

    public ObservedFeature(Vector2 feature) {
        this.feature = feature;
        observedCount = 1;
    }
}

public abstract class IFeature {
    public int index;

    public abstract bool IsFeature();
    public abstract float Magnitude();
}

public class Feature: IFeature {
    public Vector2 feature;
    private RobotPose parent;//Parent will not be serialized.

    //Should be used only for the network messaging.
    public Feature() { }

    public Feature(Vector2 f, RobotPose p, int index) {
        feature = f;
        parent = p;
    }
    public override bool IsFeature() { return true; }
    public override float Magnitude() { return feature.magnitude; }
    public RobotPose ParentPose() { return parent; }
}

public class RobotPose: IFeature {
    public static RobotPose zero = new RobotPose(Vector3.zero, 0f);

    public Vector3 pose;
    public float radius;//The radius of the local map.

    //Should be used only for the network messaging.
    public RobotPose() { }

    public RobotPose(Vector3 f, float r) {
        pose = f;
        radius = r;
    }
    public override bool IsFeature() { return false; }
    public override float Magnitude() { return pose.magnitude; }

    public static Vector2 operator +(RobotPose a, Vector2 b) {
        return new Vector2(a.pose.x + b.x, a.pose.y + b.y);
    }
}

public class FeatureEnumerator : IEnumerator<Feature> {

    private IEnumerator<IFeature> features;

    public FeatureEnumerator(List<IFeature> features) {
        this.features = features.GetEnumerator();
    }

    public Feature Current {
        get { return (Feature) features.Current; }
    }

    object IEnumerator.Current {
        get { return features.Current; }
    }

    public void Dispose() {
        features.Dispose();
        features = null;
    }

    public bool MoveNext() {
        bool b = features.MoveNext();
        while (b) {
            if (features.Current.IsFeature()) break;
            b = features.MoveNext();
        }
        return b;
    }

    public void Reset() {
        features.Reset();
    }
}

//Same as the FeatureEnumerator but it returns the Vector2 in each feature.
public class FeatureVectorEnumerator : IEnumerator {

    private IEnumerator<IFeature> features;

    public FeatureVectorEnumerator(List<IFeature> features) {
        this.features = features.GetEnumerator();
    }

    public object Current {
        get { return ((Feature) features.Current).feature; }
    }

    public bool MoveNext() {
        bool b = features.MoveNext();
        while (b) {
            if (features.Current.IsFeature()) break;
            b = features.MoveNext();
        }
        return b;
    }

    public void Reset() {
        features.Reset();
    }
}

public class FeatureListVectorEnumerator : IEnumerator {

    private IEnumerator<List<Feature>> featureLists;
    private IEnumerator<Feature> features;

    public FeatureListVectorEnumerator(List<List<Feature>> featureLists) {
        this.featureLists = featureLists.GetEnumerator();
        this.featureLists.MoveNext();
        features = this.featureLists.Current.GetEnumerator();
    }

    public object Current {
        get { return features.Current.feature; }
    }

    public bool MoveNext() {
        bool b = features.MoveNext();
        while(!b) {
            if(featureLists.MoveNext()) {
                features = featureLists.Current.GetEnumerator();
                b = features.MoveNext();
            }
            else return false;
        }
        return true;
    }

    public void Reset() {
        featureLists.Reset();
        featureLists.MoveNext();
        features = featureLists.Current.GetEnumerator();
    }
}

public class PrematchFeatureEnumerator : IEnumerator<Feature> {

    private List<IFeature> features;
    private IEnumerator<int> featureIndexes;

    internal PrematchFeatureEnumerator(List<IFeature> features, IEnumerable<int> featureIndexes) {
        this.features = features;
        this.featureIndexes = featureIndexes.GetEnumerator();
    }

    public Feature Current {
        get { return (Feature)features[featureIndexes.Current]; }
    }

    object IEnumerator.Current {
        get { return features[featureIndexes.Current]; }
    }

    public void Dispose() {
        featureIndexes.Dispose();
        features = null;
        featureIndexes = null;
    }

    public bool MoveNext() {
        return featureIndexes.MoveNext();
    }

    public void Reset() {
        featureIndexes.Reset();
    }
}

public class CombinedFeatureEnumerator : IEnumerator<Feature> {

    private bool isFirstSelected = true;
    private IEnumerator features1;
    private IEnumerator<ObservedFeature> features2;
    private int features1Count;
    private int i = 0;
    private Feature current;

    internal CombinedFeatureEnumerator(IEnumerator features1, int features1Count, IEnumerator<ObservedFeature> features2) {
        this.features1 = features1;
        this.features2 = features2;
        this.features1Count = features1Count;
    }

    public Feature Current {
        get { return current; }
    }

    object IEnumerator.Current {
        get { return current; }
    }

    public void Dispose() {
        features1 = null;
        features2 = null;
        current = null;
    }

    public bool MoveNext() {
        bool b;
        if (isFirstSelected) {
            i++;
            b = features1.MoveNext();
            if (!b || i >= features1Count) {
                isFirstSelected = false;
                i = -1;
                b = features2.MoveNext();
            }
        } else {
            i--;
            b = features2.MoveNext();
        }
        if (!b) return false;
        current = new Feature(isFirstSelected ? (Vector2) features1.Current : features2.Current.feature, RobotPose.zero, i);
        return true;
    }

    public void Reset() {
        i = 0;
        features1.Reset();
        features2.Reset();
    }
}

public interface IArray<T> {
    T this[int i] {
        get;
        set;
    }
    int Count {
        get;
    }
}

public class VectorArray : IArray<Vector2> {

    private Vector2[] arr;

    public VectorArray(Vector2[] arr) {
        this.arr = arr;
    }

    public Vector2 this[int i] {
        get { return arr[i]; }
        set { arr[i] = value; }
    }

    public int Count {
        get { return arr.Length; }
    }
}

public class FeatureVectorArray: IArray<Vector2> {

    private List<IFeature> list;

    public FeatureVectorArray(List<IFeature> list) {
        this.list = list;
    }

    public Vector2 this[int i] {
        get { return ((Feature) list[i]).feature; }
        set { ((Feature)list[i]).feature = value; }
    }

    public int Count {
        get { return list.Count; }
    }
}
