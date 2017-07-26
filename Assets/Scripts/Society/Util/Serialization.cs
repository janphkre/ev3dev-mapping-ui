using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
class IntList : List<int> { }

[Serializable]
class MatrixList : List<Matrix> { }

[Serializable]
public class SparseColumnList : List<SparseColumn> { }

[Serializable]
public class IntMatrixDictionary : ISerializationCallbackReceiver, IEnumerable<KeyValuePair<int, Matrix>> {

    //Dictionaries have to be serialized manually.
    private Dictionary<int, Matrix> val = new Dictionary<int, Matrix>();
    [SerializeField] private IntList keys;
    [SerializeField] private MatrixList values;

    public void OnAfterDeserialize() {
        var keyEnumerator = keys.GetEnumerator();
        var valuesEnumerator = values.GetEnumerator();
        while(keyEnumerator.MoveNext()) {
            valuesEnumerator.MoveNext();
            val.Add(keyEnumerator.Current, valuesEnumerator.Current);
        }
    }

    public void OnBeforeSerialize() {
        keys = new IntList();
        values = new MatrixList();
        foreach(KeyValuePair<int, Matrix> pair in val) {
            keys.Add(pair.Key);
            values.Add(pair.Value);
        }
    }

    public void Add(int key, Matrix value) {
        val.Add(key, value);
    }

    public bool TryGetValue(int key, out Matrix value) {
        return val.TryGetValue(key, out value);
    }

    public Matrix this[int i] {
        get { return val[i]; }
        set { val[i] = value; }
    }

    public bool Remove(int key) {
        return val.Remove(key);
    }

    public void Clear() {
        val.Clear();
    }

    public IEnumerator<KeyValuePair<int, Matrix>> GetEnumerator() {
        return val.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator() {
        return val.GetEnumerator();
    }

    public Dictionary<int, Matrix>.KeyCollection Keys {
        get { return val.Keys; }
    }
}

[Serializable]
public class IFeatureList : List<IFeature> { }