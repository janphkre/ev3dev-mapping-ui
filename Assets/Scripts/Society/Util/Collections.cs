using System;
using System.Collections.Generic;
using UnityEngine;

public class MatrixSizeException : Exception {
    public MatrixSizeException(int sizeAX, int sizeBX, int sizeAY, int sizeBY, String op) : base("Sizes of Matrices do not match:" + sizeAX + ", " + sizeAY + op + sizeBX + ", " + sizeBY) { }
    public MatrixSizeException(int sizeAX, int sizeBY, String op) : base("Sizes of Matrices do not match:" + sizeAX + op + sizeBY) { }
}

public class PointCollection {
    public Vector4[] map;
    public Vector3 end;

    public PointCollection() { }

    public PointCollection(PointCollection otherMap) {
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

public class CovarianceMatrix {

    public List<Row> val = new List<Row>();
    public int count;

    public CovarianceMatrix() {//TODO: first matrix should include initial error for the robot pose
        count = 1;
        val.Add(new Row());
    }

    public CovarianceMatrix(int i) {
        val.Add(new Row());
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

public class Matrix {
    private float[,] val;
    public int sizeX;
    public int sizeY;

    public Matrix(int size) {
        this.sizeX = size;
        this.sizeY = size;
        val = new float[sizeX, sizeY];
        for(int i = 0; i < sizeX; i++) {
            val[i, i] = 1.0f;
        }
    }

    public Matrix(int sizeX, int sizeY) {
        this.sizeX = sizeX;
        this.sizeY = sizeY;
        val = new float[sizeX, sizeY];
    }

    public float this[int i, int j] {
        get { return val[i, j]; }
        set { val[i, j] = value; }
    }

    public static Matrix operator +(Matrix a, Matrix b) {
        if (a.sizeX != b.sizeX || a.sizeY != b.sizeY) throw new MatrixSizeException(a.sizeX, b.sizeX, a.sizeY, b.sizeY, "+");
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[i, j] + b[i, j];
            }
        }
        return result;
    }

    public static Matrix operator *(Matrix a, Matrix b) {
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
        Matrix result = new Matrix(a.sizeX);
        for (int i = 0; i < result.sizeX - 1; i++) {
            for (int j = i + 1; j < result.sizeX - 1; j++) {
                float s = a[j, i] / a[i, i];
                for (int k = i; k < result.sizeX; k++) {
                    a[j, k] -= a[i, k] * s;
                }
                for(int k = 0; k < result.sizeX; k++) {
                    result[j, k] -= result[i, k] * s;
                }
            }
        }
        for (int i = result.sizeX - 1; i >= 0; i--) {
            result[i,i] /= a[i, i];
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