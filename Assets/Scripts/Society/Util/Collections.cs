using Superbest_random;
using System;
using System.Collections.Generic;
using UnityEngine;

public class MatrixSizeException : Exception {
    public MatrixSizeException(int sizeAX, int sizeBX, int sizeAY, int sizeBY, String op) : base("A size of a matrix is wrong:" + sizeAX + ", " + sizeAY + op + sizeBX + ", " + sizeBY) { }
    public MatrixSizeException(int sizeAX, int sizeBY, String op) : base("A size of a matrix is wrong:" + sizeAX + op + sizeBY) { }
}

public class FeatureCollection {
    //The map consists of features. Each feature is a line. x,y form the start of the line, z,w the end of the line. x <= z should be valid.
    public Vector4[] map;
    public Vector2 end;
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

public class Matrix {
    private float[,] val;
    public int sizeX;
    public int sizeY;

    public Matrix(int size) {
        this.sizeX = size;
        this.sizeY = size;
        val = new float[sizeX, sizeY];
        for (int i = 0; i < sizeX; i++) {
            val[i, i] = 1.0f;
        }
    }

    public Matrix(int sizeX, int sizeY) {
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        val = new float[sizeX, sizeY];
    }

    public void Empty() {
        for (int i = 0; i < sizeX; i++) for (int j = 0; j < sizeY; j++) val[i, j] = 0f;
    }

    public float this[int i, int j] {
        get { return val[i, j]; }
        set { val[i, j] = value; }
    }

    public static Matrix operator +(Matrix a, DiagonalMatrix b) {
        if (a.sizeX != b.Size() || a.sizeY != b.Size()) throw new MatrixSizeException(a.sizeX, b.Size(), a.sizeY, b.Size(), "+");
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[i, j];
                if (i == j) result[i, j] += b[i];
            }
        }
        return result;
    }

    public static Matrix operator *(Matrix a, DiagonalMatrix b) {
        if (a == null) return b.ToMatrix();
        if (b == null) return a;
        if (a.sizeX != b.Size()) throw new MatrixSizeException(a.sizeX, b.Size(), "*");
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[i, j] * b[j];
            }
        }
        return result;
    }

    public static Matrix operator +(Matrix a, Matrix b) {
        if (a == null) return b;
        if (b == null) return a;
        if (a.sizeX != b.sizeX || a.sizeY != b.sizeY) throw new MatrixSizeException(a.sizeX, b.sizeX, a.sizeY, b.sizeY, "+");
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[i, j] + b[i, j];
            }
        }
        return result;
    }

    public static Matrix operator -(Matrix a) {
        if (a == null) return null;
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = -a[i, j];
            }
        }
        return result;
    }

    public static Matrix operator -(Matrix a, Matrix b) {
        if (b == null) return a;
        if (a == null) return -b;
        if (a.sizeX != b.sizeX || a.sizeY != b.sizeY) throw new MatrixSizeException(a.sizeX, b.sizeX, a.sizeY, b.sizeY, "-");
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[i, j] - b[i, j];
            }
        }
        return result;
    }

    public static Matrix operator *(Matrix a, Matrix b) {
        if (a == null || b == null) return null;
        if (a.sizeX != b.sizeY) throw new MatrixSizeException(a.sizeX, b.sizeY, "*");
        Matrix result = new Matrix(b.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = 0.0f;
                for (int k = 0; k < a.sizeX; k++) {
                    result[i, j] += a[i, k] * b[k, j];
                }
            }
        }
        return result;
    }

    //translate
    public static Matrix operator ~(Matrix a) {
        Matrix result = new Matrix(a.sizeY, a.sizeX);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[j, i];
            }
        }
        return result;
    }

    //inverse
    //https://gist.github.com/occar421/feb03a0183e69ecc0189
    public static Matrix operator !(Matrix a) {
        if (a.sizeX != a.sizeY) throw new MatrixSizeException(a.sizeX, a.sizeY, "!");
        Matrix result = new Matrix(a.sizeX);
        for (int i = 0; i < result.sizeX - 1; i++) {
            for (int j = i + 1; j < result.sizeX - 1; j++) {
                float s = a[j, i] / a[i, i];
                for (int k = i; k < result.sizeX; k++) {
                    a[j, k] -= a[i, k] * s;
                }
                for (int k = 0; k < result.sizeX; k++) {
                    result[j, k] -= result[i, k] * s;
                }
            }
        }
        for (int i = result.sizeX - 1; i >= 0; i--) {
            result[i, i] /= a[i, i];
            a[i, i] /= a[i, i];
            for (int j = i - 1; j >= 0; j--) {
                float s = a[j, i] / a[i, i];
                a[j, i] -= s;
                for (int k = 0; k < result.sizeX; k++) {
                    result[j, k] -= result[i, k] * s;
                }
            }
        }
        return result;
    }
}

public class Row {
    public List<Matrix> val = new List<Matrix>();
    public int sizeY;

    public Row() {
        sizeY = 3;
        val.Add(new Matrix(3));
    }

    public Row(int count) {
        sizeY = 2;
        val.Add(new Matrix(3, 2));
        for (int i = 1; i < count - 1; i++) val.Add(new Matrix(2, 2));
        val.Add(new Matrix(2));
    }

    public void Enlarge(int count) {
        for (int i = 0; i < count; i++) val.Add(new Matrix(sizeY, 2));
    }

}

//It is very likely that an Exception will occur. This is because the foreach is commented out in Enlarge.
//The access onto the covariance matrix has to be edited because it is only the lower left triangle at the moment.
public class CovarianceMatrix {

    private const float INITIAL_ERROR = 0.1f; 

    public List<Row> val = new List<Row>();
    public int count;

    public CovarianceMatrix(System.Random random) {//TODO: first matrix should include initial error for the robot pose
        count = 1;
        val.Add(new Row());
        val[0].val[0][0, 0] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][1, 1] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][2, 2] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
    }

    public CovarianceMatrix(System.Random random, int i) {
        val.Add(new Row());
        val[0].val[0][0, 0] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][1, 1] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][2, 2] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        for (count = 2; count <= i; count++) val.Add(new Row(count));
        count--;
    }

    public void Enlarge(int i) {
        /*foreach (Row row in val) {
            row.Enlarge(i);
        }*/
        for (int j = 0; j <= i; j++) val.Add(new Row(++count));
    }

    public void AddRow(Row row) {
        val.Add(row);
        ++count;
    }

    public Matrix this[int i, int j] {
        get { return val[i].val[j]; }
        set { val[i].val[j] = value; }
    }

    /*public Row this[int i] {
        get { return val[i]; }
        set { val[i] = value; }
    }*/

    public static Matrix operator *(CovarianceMatrix a, Matrix b) {
        if (a.count != b.sizeY) throw new MatrixSizeException(a.count, b.sizeY, "*");
        Matrix result = new Matrix(b.sizeX, a.count);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = 0.0f;
                for (int k = 0; k < b.sizeY; k++) {
                    if (i < 3) {
                        if (k < 3) {
                            result[i, j] += a[0, 0][i, k] * b[k, j];
                        } else {
                            result[i, j] += a[0, (k - 3) / 2][i, (k - 3) % 2] * b[k, j];
                        }
                    } else {
                        if (k < 3) {
                            result[i, j] += a[(i - 3) / 2, 0][(i - 3) % 2, k] * b[k, j];
                        } else {
                            result[i, j] += a[(i - 3) / 2, (k - 3) / 2][(i - 3) % 2, (k - 3) % 2] * b[k, j];
                        }
                    }
                }
            }
        }
        return result;
    }
}

public class DiagonalMatrix {
    private List<float> val = new List<float>();

    public DiagonalMatrix() { }

    public void Enlarge(int i) {
        for (int j = 0; j <= i; j++) val.Add(1.0f);
    }

    public float this[int i] {
        get { return val[i]; }
        set { val[i] = value; }
    }

    public int Size() {
        return val.Count;
    }

    public static DiagonalMatrix operator +(DiagonalMatrix a, DiagonalMatrix b) {
        if (a.val.Count != b.val.Count) throw new MatrixSizeException(a.val.Count, b.val.Count, "*");
        DiagonalMatrix result = new DiagonalMatrix();
        for (int i = 0; i < a.val.Count; i++) {
            result.val.Add(a[i] + b[i]);
        }
        return result;
    }

    public static DiagonalMatrix operator *(DiagonalMatrix a, DiagonalMatrix b) {
        if (a.val.Count != b.val.Count) throw new MatrixSizeException(a.val.Count, b.val.Count, "*");
        DiagonalMatrix result = new DiagonalMatrix();
        for (int i = 0; i < a.val.Count; i++) {
            result.val.Add(a[i] * b[i]);
        }
        return result;
    }

    //translate
    public static DiagonalMatrix operator ~(DiagonalMatrix a) {
        return a;
    }

    //inverse
    public static DiagonalMatrix operator !(DiagonalMatrix a) {
        DiagonalMatrix result = new DiagonalMatrix();
        foreach(float f in a.val) {
            result.val.Add(1f/f);
        }
        return result;
    }

    public Matrix ToMatrix() {
        Matrix result = new Matrix(val.Count);
        for (int i = 0; i < val.Count; i++) {
            result[i,i] = val[i];
        }
        return result;
    }
}

public class SparseColumn {
    //public List<int> indices = new List<int>();
    //public List<Matrix> val = new List<Matrix>();
    public Dictionary<int, Matrix> val = new Dictionary<int, Matrix>();
    public readonly int size;

    public SparseColumn(int i) {
        size = i;
    }

    public Matrix this[int j] {
        get {
            Matrix result;
            if (val.TryGetValue(j, out result)) return result;
            return null;
        }
        set { val[j] = value; }
    }

    public void Remove(int i) {
        val.Remove(i);
    }

    public void Clear() {
        val.Clear();
    }
}

public class SparseCovarianceMatrix {
    private List<SparseColumn> val = new List<SparseColumn>();
    private int count = 0;

    public SparseCovarianceMatrix() { }

    public void Enlarge2(int i) {
        count += i * 2;
        while (i-- != 0) val.Add(new SparseColumn(2));
    }

    //Enlarges the sparse covariance matrix by a new row for a 3x3 matrix (a robot pose)
    public void Enlarge3() {
        val.Add(new SparseColumn(3));
        count += 3;
    }

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    //Returns the matrix size for the i-th column/row.
    public int this[int i] {
        get { return val[i].size; }
    }

    public int ColumnCount() {
        return val.Count;
    }

    //Returns the summed count of the submatrices' sizes (=the "real" matrix size).
    public int Count() {
        return count;
    }

    public void Add(SparseColumn col) {
        val.Add(col);
        count += col.size;
    }

    //KeepRows must be sorted!
    public void Trim(List<int> keepRows, int size) {
        foreach (SparseColumn col in val) {
            List<int>.Enumerator enumerator = keepRows.GetEnumerator();
            if (!enumerator.MoveNext()) throw new ArgumentException("keepRows is empty");
            int j = 0;
            for (int i = 0; i < size - 1; i++) {
                //After the last element in keepRows enumerartor.Current is Undefined. MoveNext returned false.
                if (enumerator.Current == i) {
                    if (col[i] != null) {
                        col[j++] = col[i];
                        col.Remove(i);
                    }
                    if (!enumerator.MoveNext()) {
                        for (i++; i < size - 1; i++) {
                            col.Remove(i);
                        }
                        break;
                    }
                }
                col.Remove(i);
            }
            //Move last row:
            if (col[size-1] != null) {
                col[j] = col[size - 1];
                col.Remove(size - 1);
            }
            enumerator.Dispose();
        }
    }
}

public class SparseTriangularMatrix {
    private List<SparseColumn> val = new List<SparseColumn>();
    private int count = 0;

    public SparseTriangularMatrix() { }

    public void Enlarge2(int i) {
        count += i * 2;
        while (i-- != 0) val.Add(new SparseColumn(2));
    }

    //Enlarges the sparse covariance matrix by a new row for a 3x3 matrix (a robot pose)
    public void Enlarge3() {
        val.Add(new SparseColumn(3));
        count += 3;
    }

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    //Returns the matrix size for the i-th column/row.
    public int this[int i] {
        get { return val[i].size; }
    }

    public int ColumnCount() {
        return val.Count;
    }

    //Returns the summed count of the submatrices' sizes (=the "real" matrix size).
    public int Count() {
        return count;
    }
}