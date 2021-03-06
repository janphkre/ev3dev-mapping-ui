﻿using Superbest_random;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

public class MatrixException : Exception {
    public MatrixException(string s) : base(s) { }
}

public class MatrixSizeException : MatrixException {
    public MatrixSizeException(int sizeAX, int sizeBX, int sizeAY, int sizeBY, string op) : base("A size of a matrix is wrong:" + sizeAX + ", " + sizeAY + op + sizeBX + ", " + sizeBY) { }
    public MatrixSizeException(int sizeAX, int sizeBY, string op) : base("A size of a matrix is wrong:" + sizeAX + op + sizeBY) { }
}

public abstract class AMatrix {
    
    protected abstract float getFloat(int i, int j);
    protected abstract void setFloat(int i, int j, float f);
    
    protected abstract int getSizeX();
    protected abstract int getSizeY();
    
    //inverse
    // See: https://github.com/DownMoney/Matrices_Inverse
    //TODO: REMOVE NECESSITY TO CLONE BEFORE THIS METHOD!
    protected void inverseTo(AMatrix result) {
        int r;
        float scale;
        //Process the matrix one column at a time
        for (int c = 0; c < getSizeX(); ++c) {
            //Swap rows if the current value is too close to 0.0
            if (Mathf.Abs(getFloat(c, c)) <= 2.0f * float.Epsilon) {
                for (r = c + 1; r < getSizeX(); ++r) if (Mathf.Abs(getFloat(r, c)) <= 2.0f * float.Epsilon) {
                        RowSwap(c, r);
                        result.RowSwap(c, r);
                        break;
                    }
                if (r >= getSizeX()) throw new MatrixException("Matrix is not invertible:\n" + ToString());
            }
            //Scale the current row to start with 1
            scale = 1.0f / getFloat(c, c);
            RowScale(scale, c);
            result.RowScale(scale, c);
            
            //Zero out the rest of the column
            for (r = 0; r < getSizeX(); ++r) {
                if (r != c) {
                    scale = -getFloat(r, c);
                    RowScaleAdd(scale, c, r);
                    result.RowScaleAdd(scale, c, r);
                }
            }
        }
    }

    /// <summary>
    /// Swap 2 rows in a matrix.
    /// </summary>
    /// <param name="data">The matrix to operate on.</param>
    /// <param name="cols">
    /// The number of columns in <paramref name="data"/>.
    /// </param>
    /// <param name="r0">One of the 2 rows to swap.</param>
    /// <param name="r1">One of the 2 rows to swap.</param>
    private void RowSwap(int r0, int r1) {
        float tmp;
        for (int i = 0; i < getSizeX(); ++i) {
            tmp = getFloat(r0, i);
            setFloat(r0, i, getFloat(r1, i));
            setFloat(r1, i, tmp);
        }
    }

    /// <summary>
    /// Perform scale and add a row in a matrix to another
    /// row:  data[r1,] = a*data[r0,] + data[r1,].
    /// </summary>
    /// <param name="data">The matrix to operate on.</param>
    /// <param name="cols">
    /// The number of columns in <paramref name="data"/>.
    /// </param>
    /// <param name="a">
    /// The scale factor to apply to row <paramref name="r0"/>.
    /// </param>
    /// <param name="r0">The row to scale.</param>
    /// <param name="r1">The row to add and store to.</param>
    private void RowScaleAdd(float a, int r0, int r1) {
        for (int i = 0; i < getSizeX(); ++i) setFloat(r1, i, getFloat(r1, i) + a * getFloat(r0, i));
    }

    /// <summary>
    /// Scale a row in a matrix by a constant factor.
    /// </summary>
    /// <param name="data">The matrix to operate on.</param>
    /// <param name="cols">The number of columns in the matrix.</param>
    /// <param name="a">
    /// The factor to scale row <paramref name="r"/> by.
    /// </param>
    /// <param name="r">The row to scale.</param>
    private void RowScale(float a, int r) {
        for (int i = 0; i < getSizeX(); ++i) setFloat(r, i, getFloat(r, i) * a);
    }

    public override bool Equals(object other) {
        if(other == null) return false;
        Type t = other.GetType();
        while(t != typeof(AMatrix)) {
            t = t.BaseType;
            if(t == null) return false;
        }
        AMatrix o = (AMatrix) other;
        if(getSizeX() != o.getSizeX() || getSizeY() != o.getSizeY()) return false;
        for (int i = 0; i < o.getSizeX(); i++)
            for (int j = 0; j < o.getSizeY(); j++) if (!Geometry.CompareFloats(getFloat(i, j), o.getFloat(i, j))) return false;
        return true;
    }

    public override int GetHashCode() {
        return base.GetHashCode();
    }

    public override string ToString() {
        string s = "";
        for(int i = 0; i < getSizeY(); i++) {
            if(i != 0) s += ", ";
            s += "[";
            for (int j = 0; j < getSizeX(); j++) {
                if(j != 0) s += ", ";
                s+= getFloat(j, i);
            }
            s += "]";
        }
        return s;
    }
}

public class Matrix : AMatrix {

    private float[,] val;
    public int sizeX;
    public int sizeY;

    public Matrix(int size) {
        sizeX = size;
        sizeY = size;
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

    public bool IsEmpty() {
        for (int i = 0; i < sizeX; i++) for (int j = 0; j < sizeY; j++) if(val[i, j] != 0f) return false;
        return true;
    }

    public void Empty() {
        for (int i = 0; i < sizeX; i++) for (int j = 0; j < sizeY; j++) val[i, j] = 0f;
    }

    public float this[int i, int j] {
        get { return val[i, j]; }
        set { val[i, j] = value; }
    }

    protected override float getFloat(int i, int j) {
        return this[i, j];
    }

    protected override void setFloat(int i, int j, float f) {
        this[i, j] = f;
    }
    
    protected override int getSizeX() { return sizeX; }
    protected override int getSizeY() { return sizeY; }
    
    public static Matrix operator +(Matrix a, DiagonalMatrix b) {
        if(a == null) return b.ToMatrix();
        if(b == null) return a;
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
        if (a == null || b == null) return null;
        if (a.sizeX != b.Size()) throw new MatrixSizeException(a.sizeX, b.Size(), "*");
        Matrix result = new Matrix(a.sizeX, a.sizeY);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[i, j] * b[i];
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
        for (int i = 0; i < result.sizeY; i++) {
            for (int j = 0; j < result.sizeX; j++) {
                for (int k = 0; k < a.sizeX; k++) {
                    result[j, i] += a[k, i] * b[j, k];
                }
            }
        }
        return result;
    }

    public static Matrix operator *(Matrix a, Vector3 b) {
        if (a == null) return null;
        if (a.sizeX != 3) throw new MatrixSizeException(a.sizeX, 3, "*");
        Matrix result = new Matrix(1, a.sizeY);
        for (int j = 0; j < result.sizeY; j++) {
            result[0, j]  = a[0, j] * b.x + a[1, j] * b.y + a[2, j] * b.z;
        }
        return result;
    }

    public static Matrix operator *(Vector2 a, Matrix b) {
        if (b == null) return null;
        if (b.sizeY != 2) throw new MatrixSizeException(2, b.sizeX, "*");
        Matrix result = new Matrix(b.sizeX, 1);
        for (int j = 0; j < result.sizeX; j++) {
            result[j, 0] = a.x * b[j, 0] + a.y * b[j, 1];
        }
        return result;
    }

    public static Matrix operator *(Matrix a, Vector2 b) {
        if (a == null) return null;
        if (a.sizeX != 2) throw new MatrixSizeException(a.sizeX, 2, "*");
        Matrix result = new Matrix(1, a.sizeY);
        for (int j = 0; j < result.sizeY; j++) {
            result[0, j] = a[0, j] * b.x + a[1, j] * b.y;
        }
        return result;
    }

    //translate
    public static Matrix operator ~(Matrix a) {
        if(a == null) return null;
        Matrix result = new Matrix(a.sizeY, a.sizeX);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = a[j, i];
            }
        }
        return result;
    }

    //inverse
    public static Matrix operator !(Matrix a) {
        if(a == null) return null;
        if (a.sizeX != a.sizeY) throw new MatrixSizeException(a.sizeX, a.sizeY, "!");
        if(a.IsEmpty()) return null;
        Matrix result = new Matrix(a.sizeX);
        a.inverseTo(result);
        return result;
    }

    public Matrix Duplicate() {
        Matrix result = new Matrix(sizeX, sizeY);
        for (int i = 0; i < result.sizeX; i++)
            for (int j = 0; j < result.sizeY; j++) result[i, j] = val[i, j];
        return result;
    }
}

public class DiagonalMatrix {
    private List<float> val = new List<float>();

    public DiagonalMatrix() { }

    public void Enlarge(int i) {
        for (int j = 0; j < i; j++) val.Add(1.0f);
    }

    public float this[int i] {
        get { return val[i]; }
        set { val[i] = value; }
    }

    public int Size() {
        return val.Count;
    }

    public static DiagonalMatrix operator +(DiagonalMatrix a, DiagonalMatrix b) {
        if(a == null) return b;
        if(b == null) return a;
        if (a.val.Count != b.val.Count) throw new MatrixSizeException(a.val.Count, b.val.Count, "+");
        DiagonalMatrix result = new DiagonalMatrix();
        for (int i = 0; i < a.val.Count; i++) {
            result.val.Add(a[i] + b[i]);
        }
        return result;
    }

    public static DiagonalMatrix operator *(DiagonalMatrix a, DiagonalMatrix b) {
        if(a == null || b == null) return null;
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

    //inverse -- Untested! (each value must not be empty for this inverse)
    /*public static DiagonalMatrix operator !(DiagonalMatrix a) {
        DiagonalMatrix result = new DiagonalMatrix();
        foreach (float f in a.val) {
            result.val.Add(1f / f);
        }
        return result;
    }*/

    public Matrix ToMatrix() {
        Matrix result = new Matrix(val.Count);
        for (int i = 0; i < val.Count; i++) {
            result[i, i] = val[i];
        }
        return result;
    }
}

public class Row {
    public List<Matrix> val = new List<Matrix>();
    public int sizeY;
    
    //This constructor is only called once for the first row (which contains 3 column matrices).
    public Row(int count, int sizeY) {
        this.sizeY = 3;
        val.Add(new Matrix(3, 3));
        for (int i = 1; i < count; i++) val.Add(new Matrix(2, 3));
    }

    public Row(int count) {
        sizeY = 2;
        val.Add(new Matrix(3, 2));
        for (int i = 1; i < count; i++) val.Add(new Matrix(2, 2));
    }

    public void Enlarge(int count) {
        for (int i = 0; i < count; i++) val.Add(new Matrix(2, sizeY));
    }

}

public interface ICovarianceMatrix {
    Matrix this[int i, int j] { get; }
}

public class CovarianceMatrix : AMatrix, ICovarianceMatrix {

    private const float INITIAL_ERROR = 0.1f;

    internal List<Row> val = new List<Row>();
    public int count = 3;
    
    private void initialize(int i) {
        count += 2 * (i - 1);
        i += val.Count;
        val.Add(new Row(i, 3));
        while(val.Count < i) val.Add(new Row(i));
    }

    public CovarianceMatrix(int i, float identity) {
        initialize(i);
        for(int j = 0; j < val.Count; j++) {
            Matrix m = this[j, j];
            for(int k = 0; k < m.sizeX; k++) m[k, k] = identity;
        }
    }

    public CovarianceMatrix(int i) {
        initialize(i);
    }

    public CovarianceMatrix(System.Random random) {
        //First matrix should include initial error for the robot pose.
        val.Add(new Row(1, 3));
        val[0].val[0][0, 0] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][1, 1] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][2, 2] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
    }

    public CovarianceMatrix(System.Random random, int i) {
        initialize(i);
        val[0].val[0][0, 0] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][1, 1] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
        val[0].val[0][2, 2] = RandomExtensions.NextGaussian(random, 1, INITIAL_ERROR);
    }

    public void Enlarge(int i) {
        foreach (Row row in val) {
            row.Enlarge(i);
        }
        count += 2 * i;
        i += val.Count;
        while(val.Count < i) val.Add(new Row(i));
    }

    public Matrix this[int i, int j] {
        get { return val[j].val[i]; }
        set { val[j].val[i] = value; }
    }

    protected override float getFloat(int i, int j) {
        int iM, iD, jM, jD;
        if(i < 3) {
            iM = i;
            iD = 0;
        } else {
            iM = (i - 3) % 2;
            iD = (i - 3 - iM) / 2 + 1;
        }
        if(j < 3) {
            jM = j;
            jD = 0;
        } else {
            jM = (j - 3) % 2;
            jD = (j - 3 - jM) / 2 + 1;
        }
        return this[iD,jD][iM,jM];
    }

    protected override void setFloat(int i, int j, float f) {
        int iM, iD, jM, jD;
        if(i < 3) {
            iM = i;
            iD = 0;
        } else {
            iM = (i - 3) % 2;
            iD = (i - 3 - iM) / 2 + 1;
        }
        if(j < 3) {
            jM = j;
            jD = 0;
        } else {
            jM = (j - 3) % 2;
            jD = (j - 3 - jM) / 2 + 1;
        }
        this[iD,jD][iM,jM] = f;
    }
    
    protected override int getSizeX() { return count; }
    protected override int getSizeY() { return count; }
    
    /*public Row this[int i] {
        get { return val[i]; }
        set { val[i] = value; }
    }*/

    //TODO fix multiplication!
    public static Matrix operator *(CovarianceMatrix a, Matrix b) {
        if (a == null || b == null) return null;
        if (a.count != b.sizeY) throw new MatrixSizeException(a.count, b.sizeY, "*");
        Matrix result = new Matrix(b.sizeX, a.count);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = 0.0f;
                for (int k = 0; k < b.sizeY; k++) {
                    int kM, kD, jM, jD;
                    if(k < 3) {
                        kM = k;
                        kD = 0;
                    } else {
                        kM = (k - 3) % 2;
                        kD = (k - 3 - kM) / 2 + 1;
                    }
                    if(j < 3) {
                        jM = j;
                        jD = 0;
                    } else {
                        jM = (j - 3) % 2;
                        jD = (j - 3 - jM) / 2 + 1;
                    }
                    result[i, j] += a[kD,jD][kM,jM] * b[i, k];
                }
            }
        }
        return result;
    }

    public static CovarianceMatrix operator *(Matrix a, CovarianceMatrix b) {
        if(a == null || b == null) return null;
        if (a.sizeX != b.count && a.sizeY != b.count) throw new MatrixSizeException(a.sizeX, b.count, "*");
        var result = new CovarianceMatrix(b.val.Count);
        for (int i = 0; i < result.val.Count; i++) {
            for (int j = 0; j < result.val.Count; j++) {
                result[i, j] = getMutlipliedCell(a, b, i, j);
            }
        }
        return result;
    }

    private static Matrix getMutlipliedCell(Matrix a, CovarianceMatrix b, int i, int j) {
        Matrix result = new Matrix(b[i, j].sizeX, b[i, j].sizeY);
        j = j == 0 ? 0 : (j - 1) * 2 + 3;
        for (int k = 0; k < result.sizeX; k++) {
            for (int l = 0; l < result.sizeY; l++) {
                for (int m = 0; m < b.val.Count; m++) {
                    var aM = m == 0 ? 0 : (m - 1) * 2 + 3;
                    for (int n = 0; n < b[m, 0].sizeX; n++) {
                        result[k, l] += a[n + aM, l+j] * b[i, m][k, n];
                    }
                }
            }
        }
        return result;
    }

    //inverse
    // See: https://github.com/DownMoney/Matrices_Inverse
    //TODO: REMOVE NECESSITY TO CLONE BEFORE THIS METHOD!
    public static CovarianceMatrix operator !(CovarianceMatrix a) {
        if(a == null) return null;
        CovarianceMatrix result = new CovarianceMatrix(a.val.Count, 1.0f);
        a.Duplicate().inverseTo(result);
        return result;
    }

    public Matrix ToMatrix() {
        Matrix result = new Matrix(count, count);
        int i = 0,
            j = 0,
            k = 0,
            l = 0;
        while(true) {
            int m = i == 0 ? 0 : (i - 1) * 2 + 3;
            int n = j == 0 ? 0 : (j - 1) * 2 + 3;
            result[m+k, n+l] = this[i, j][k, l];
            k++;
            if(i > 0) {
                if(k > 1) {
                    k = 0;
                    i++;
                    if(i >= val.Count) {
                        i = 0;
                        l++;
                        if(j > 0) {
                            if(l > 1) {
                                l = 0;
                                j++;
                                if(j >= val.Count) break;
                            }
                        } else {
                            if(l > 2) {
                                l = 0;
                                j++;
                            }
                        }
                    }
                }
            } else {
                if(k > 2) {
                    k = 0;
                    i++;
                }
            }
        }
        return result;
    }

    public CovarianceMatrix Duplicate() {
        CovarianceMatrix result = new CovarianceMatrix(val.Count);
        for(int i = 0; i < val.Count; i++) {
            for(int j = 0; j < val.Count; j++) {
                result[i, j] = this[i, j].Duplicate();
            }
        }
        return result;
    }
    }

[Serializable]
public class SparseColumn {

    internal IntMatrixDictionary val = new IntMatrixDictionary();
    //public readonly int size;

    public SparseColumn() { }

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

    public void Addition(SparseColumn addition) {
        if(addition == null) return;
        foreach(KeyValuePair<int, Matrix> pair in addition.val) {
            this[pair.Key] += pair.Value;
        }
    }

    public static SparseColumn operator +(SparseColumn a, SparseColumn b) {
        if(a == null) return b;
        if(b == null) return a;
        SparseColumn result = new SparseColumn();
        foreach(KeyValuePair<int,Matrix> pair in a.val) {
            result.val.Add(pair.Key, pair.Value + b[pair.Key]);
        }
        foreach (KeyValuePair<int, Matrix> pair in b.val) {
            if(result[pair.Key] == null) result.val.Add(pair.Key, pair.Value);
        }
        return result;
    }

    public override bool Equals(object obj) {
        if(obj == null) return false;
        if(obj.GetType() == typeof(SparseColumn)) {
            SparseColumn o = (SparseColumn) obj;
            if(val.Keys.Count != o.val.Keys.Count) {
            	return false;
            }
            foreach (int key in val.Keys) if(!this[key].Equals(o[key])) {
            	return false;
            }
            foreach (int key in o.val.Keys) if(!o[key].Equals(this[key])) {
            	return false;
            }
            return true;
        }
        return false;
    }

	public override int GetHashCode() {
        return base.GetHashCode();
	}

	public override string ToString() {
        string s = "";
        foreach(KeyValuePair<int, Matrix> pair in val) {
            s += "[" + pair.Key + ": " + pair.Value.ToString() + "]";
        }
        return s;
    }
}

public class SparseMatrix {

    internal List<SparseColumn> val = new List<SparseColumn>();
    private int rowCount = 0;

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    public void Enlarge(int i) {
        while (i-- != 0) val.Add(new SparseColumn());
    }

    public void AddColumn(SparseColumn col) {
        val.Add(col);
    }

    public SparseColumn GetColumn(int i) {
        return val[i];
    }

    public void SetRowCount(int i) {
        rowCount = i;
    }

    public int RowCount() {
        return rowCount;
    }

    public int ColumnCount() {
        return val.Count;
    }

    public static SparseColumn operator *(SparseMatrix a, List<IFeature> b) {
        if (a == null || b == null) return null;
        if(a.ColumnCount() != b.Count) throw new MatrixSizeException(a.ColumnCount(), b.Count, "*");
        SparseColumn result = new SparseColumn();
        int i = 0;
        foreach(SparseColumn col in a.val) {
            IFeature f = b[i++];
            if(f.IsFeature()) {
                Feature fF = (Feature) f;
                foreach (KeyValuePair<int, Matrix> pair in col.val) {
                    result[pair.Key] += pair.Value * fF.feature;
                }
            } else {
                RobotPose fR = (RobotPose) f;
                foreach (KeyValuePair<int, Matrix> pair in col.val) {
                    result[pair.Key] += pair.Value * fR.pose;
                }
            }
        }
        return result;
    }

    public static SparseColumn operator *(SparseMatrix a, SparseColumn b) {
        if (a == null || b == null) return null;
        SparseColumn result = new SparseColumn();//TODO:ignore size of SparseColumn
        int i = 0;
        foreach (SparseColumn col in a.val) {
            Matrix bVal = b[i];
            foreach (KeyValuePair<int, Matrix> pair in col.val) {
                result[pair.Key] += pair.Value * bVal;
            }
            i++;
        }
        return result;
    }

    public static SparseMatrix operator *(SparseMatrix a, SparseMatrix b) {
        if (a == null || b == null) return null;
        if (a.ColumnCount() != b.RowCount()) throw new MatrixSizeException(a.ColumnCount(), b.RowCount(), "*");
        SparseMatrix result = new SparseMatrix();
        result.Enlarge(b.ColumnCount());
        int count = a.RowCount();
        result.SetRowCount(count);
        int i = 0;
        foreach(SparseColumn colB in b.val) {
            for(int j = 0; j < count; j++) {
                Matrix resultField = null;
                foreach (KeyValuePair<int, Matrix> pair in colB.val) {
                        resultField += a[pair.Key, j] * pair.Value;
                }
                if (resultField != null) result[i, j] = resultField;
            }
            i++;
        }
        return result;
    }

    //translate
    public static SparseTranslatedMatrix operator ~(SparseMatrix a) {
        if(a == null) return null;
        return new SparseTranslatedMatrix(a);
    }

    public override bool Equals(object obj) {
        if(obj == null) return false;
        if(obj.GetType() == typeof(SparseMatrix)) {
            SparseMatrix o = (SparseMatrix) obj;
            if(val.Count != o.val.Count) return false;
            for (int i = 0; i < val.Count; i++) if(!val[i].Equals(o.val[i])) return false;
            return true;
        }
        return false;
    }

    public override int GetHashCode() {
        return base.GetHashCode();
    }

    public override string ToString() {
        string s = "";
        for( int i = 0; i < val.Count; i++) {
            string t = val[i].ToString();
            if(t.Equals("")) continue;
            s += t;
            if(i != val.Count - 1) s += ", ";
        }
        s += "";
        return s;
    }
}

public class SparseTranslatedMatrix {
    private SparseMatrix matrix;

    public SparseTranslatedMatrix(SparseMatrix matrix) {
        this.matrix = matrix;
    }

    public SparseColumn GetRow(int i) {
        return matrix.GetColumn(i);
    }

    public int RowCount() {
        return matrix.ColumnCount();
    }

    public int ColumnCount() {
        return matrix.RowCount();
    }

    //translate
    public static SparseMatrix operator ~(SparseTranslatedMatrix a) {
        if(a == null) return null;
        return a.matrix;
    }
}

[Serializable]
public class SparseCovarianceMatrix : ICovarianceMatrix {
    public SparseColumnList val = new SparseColumnList();
    
    public SparseCovarianceMatrix() { }

    public void Enlarge(int i) {
        while (i-- != 0) val.Add(new SparseColumn());
    }

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    public SparseColumn GetColumn(int i) {
        return val[i];
    }

    public int ColumnCount() {
        return val.Count;
    }

    internal void Clear() {
        foreach (SparseColumn col in val) col.Clear();
    }

    public void Add(SparseColumn col) {
        val.Add(col);
    }

    //KeepRows must be sorted!
    public void Trim(LinkedList<int> keepRows, int size) {
    	for(int i = size; i < val.Count; i++) {
        	val.RemoveAt(size);
        }
        foreach (SparseColumn col in val) {
            var enumerator = keepRows.GetEnumerator();
            if (!enumerator.MoveNext()) throw new ArgumentException("keepRows is empty");
            int j = 0;
            for (int i = 0; i < size; i++) {
                if (enumerator.Current == i) {
                    if (col[i] != null && i != j) {
                        col[j] = col[i];
                        col.Remove(i);
                    }
                    j++;
                    if (!enumerator.MoveNext()) {
                        for (i++; i < size; i++) {
                            col.Remove(i);
                        }
                        break;
                    }
                } else col.Remove(i);
            }
            enumerator.Dispose();
        }
    }

    public void Addition(SparseMatrix addition) {
        if(addition == null) return;
        if (ColumnCount() != addition.ColumnCount()) throw new MatrixSizeException(ColumnCount(), addition.ColumnCount(), "a");
        int i = 0;
        foreach (SparseColumn colAdd in addition.val) {
            SparseColumn col = val[i];
            foreach (KeyValuePair<int, Matrix> pair in colAdd.val) {
                col[pair.Key] += pair.Value;
            }
            i++;
        }
    }

    public static SparseMatrix operator *(SparseTranslatedMatrix a, SparseCovarianceMatrix b) {
        if (a == null || b == null) return null;
        if (a.ColumnCount() != b.ColumnCount()) throw new MatrixSizeException(a.ColumnCount(), b.ColumnCount(), "*");
        int count = a.ColumnCount();
        SparseMatrix result = new SparseMatrix();
        result.Enlarge(b.ColumnCount());
        result.SetRowCount(a.RowCount());
        for (int i = 0; i < a.RowCount(); i++) {
            SparseColumn row = a.GetRow(i);
            for (int j = 0; j < count; j++) {
                SparseColumn col = b.val[j];
                Matrix resultField = null;
                foreach (KeyValuePair<int, Matrix> pair in row.val) {
                    Matrix m;
                    if (col.val.TryGetValue(pair.Key, out m)) {
                        resultField += ~pair.Value * m;
                    }
                }
                if (resultField != null) result[j, i] = resultField;
            }
        }
        return result;
    }
    
    public override bool Equals(object obj) {
        if(obj == null) return false;
        if(obj.GetType() == typeof(SparseCovarianceMatrix)) {
            SparseCovarianceMatrix o = (SparseCovarianceMatrix) obj;
            if(val.Count != o.val.Count) return false;
            for (int i = 0; i < val.Count; i++) if(!val[i].Equals(o.val[i])) {
            	return false;
            }
            return true;
        }
        return false;
    }

	public override int GetHashCode() {
        return base.GetHashCode();
	}

	public override string ToString() {
        string s = "";
        for( int i = 0; i < val.Count; i++) {
            string t = val[i].ToString();
            if(t.Equals("")) continue;
            s += t;
            if(i != val.Count - 1) s += ", ";
        }
        s += "";
        return s;
    }
}

public class SparseTriangularMatrix {
    private List<SparseColumn> val = new List<SparseColumn>();

    public SparseTriangularMatrix() { }

    public void Enlarge(int i) {
        while (i-- != 0) val.Add(new SparseColumn());
    }

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    public int ColumnCount() {
        return val.Count;
    }

    public IEnumerator<SparseColumn> GetColumnEnumerator() {
        return val.GetEnumerator();
    }

    public SparseColumn solveLowerLeftSparse(SparseColumn rightHandSide) {
        SparseColumn result = new SparseColumn();
        for (int i = 0; i < val.Count; i++) {
            if (this[i, i] != null) {
                Matrix m = rightHandSide[i];
                for (int j = 0; j < i; j++) {
                    m -= this[j, i] * result[j];
                }
                if (m != null) result[i] = !(this[i, i].Duplicate()) * m;
            }
        }
        return result;
    }

    public SparseColumn solveUpperRightSparse(SparseColumn rightHandSide) {
        SparseColumn result = new SparseColumn();
        for (int i = val.Count - 1; i >= 0; i--) {
            if (this[i, i] != null) {
                Matrix m = rightHandSide[i];
                foreach(KeyValuePair<int, Matrix> pair in val[i].val) {
                    m -= ~pair.Value * result[pair.Key];
                }
                if (m != null) result[i] =!~(this[i, i].Duplicate()) * m;
            }
        }
        return result;
    }
}

public class DefaultedSparseFloatMatrix {

    private List<SparseFloatColumn> val = new List<SparseFloatColumn>();
    private float def;

    public DefaultedSparseFloatMatrix(float def) {
        this.def = def;
    }

    public float this[int i, int j] {
        get { return val[i][j]; }
        set {  val[i][j] = value; }
    }

    public IEnumerator<KeyValuePair<int, float>> GetColumn(int i) {
        return val[i].val.GetEnumerator();
    }

    public void Enlarge(int i) {
        while (i-- != 0) val.Add(new SparseFloatColumn(def));
    }

    public int ColumnCount() {
        return val.Count;
    }
}

public class SparseFloatColumn {

    public Dictionary<int, float> val = new Dictionary<int, float>();
    private readonly float def;

    public SparseFloatColumn(float def) {
        this.def = def;
    }

    public float this[int j] {
        get {
            float result;
            if (val.TryGetValue(j, out result)) return result;
            return def;
        }
        set { val[j] = value; }
    }
}
}
