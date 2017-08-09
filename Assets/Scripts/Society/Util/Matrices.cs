using Superbest_random;
using System;
using System.Collections.Generic;
using UnityEngine;

public class MatrixSizeException : Exception {
    public MatrixSizeException(int sizeAX, int sizeBX, int sizeAY, int sizeBY, String op) : base("A size of a matrix is wrong:" + sizeAX + ", " + sizeAY + op + sizeBX + ", " + sizeBY) { }
    public MatrixSizeException(int sizeAX, int sizeBY, String op) : base("A size of a matrix is wrong:" + sizeAX + op + sizeBY) { }
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
                    result[i, j] += a[k, i] * b[j, k];
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
            result[0, j]  = a[j, 0] * b.x + a[j, 1] * b.y + a[j, 2] * b.z;
        }
        return result;
    }

    public static Matrix operator *(Vector2 a, Matrix b) {
        if (b == null) return null;
        if (b.sizeY != 2) throw new MatrixSizeException(2, b.sizeX, "*");
        Matrix result = new Matrix(b.sizeX, 1);
        for (int j = 0; j < result.sizeY; j++) {
            result[0, j] = a.x * b[0, j] + a.y * b[1, j];
        }
        return result;
    }

    public static Matrix operator *(Matrix a, Vector2 b) {
        if (a == null) return null;
        if (a.sizeX != 2) throw new MatrixSizeException(a.sizeX, 2, "*");
        Matrix result = new Matrix(1, a.sizeY);
        for (int j = 0; j < result.sizeY; j++) {
            result[0, j] = a[j, 0] * b.x + a[j, 1] * b.y;
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

    public Matrix Clone() {
        Matrix result = new Matrix(sizeX, sizeY);
        for (int i = 0; i < result.sizeX; i++)
            for (int j = i + 1; j < result.sizeY; j++) result[i, j] = val[i, j];
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
        if (a.val.Count != b.val.Count) throw new MatrixSizeException(a.val.Count, b.val.Count, "+");
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
        foreach (float f in a.val) {
            result.val.Add(1f / f);
        }
        return result;
    }

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

    public Row() {
        sizeY = 3;
        val.Add(new Matrix(3, 3));
    }

    public Row(int count) {
        sizeY = 2;
        val.Add(new Matrix(3, 2));
        for (int i = 1; i < count; i++) val.Add(new Matrix(2, 2));
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

    public CovarianceMatrix(int i) {
        val.Add(new Row());
        for (count = 2; count <= i; count++) val.Add(new Row(count));
        count--;
    }

    public CovarianceMatrix(System.Random random) {
        //First matrix should include initial error for the robot pose.
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

    //TODO fix multiplication!
    public static Matrix operator *(CovarianceMatrix a, Matrix b) {
        if (a.count != b.sizeY) throw new MatrixSizeException(a.count, b.sizeY, "*");
        Matrix result = new Matrix(b.sizeX, a.count);
        for (int i = 0; i < result.sizeX; i++) {
            for (int j = 0; j < result.sizeY; j++) {
                result[i, j] = 0.0f;
                for (int k = 0; k < b.sizeY; k++) {
                    if (i < 3) {
                        if (k < 3) {
                            result[i, j] += a[0, 0][k, i] * b[j, k];
                        } else {
                            result[i, j] += a[(k - 3) / 2, 0][i, (k - 3) % 2] * b[k, j];
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

    public static CovarianceMatrix operator *(Matrix a, CovarianceMatrix b) {
        if (a.sizeY != b.count) throw new MatrixSizeException(a.sizeY, b.count, "*");
        var result = new CovarianceMatrix(b.count);
        for (int i = 0; i < result.count; i++) {
            for (int j = 0; j <= i; j++) {
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
                for (int m = 0; m < b.count; m++) {
                    var aM = m == 0 ? 0 : (m - 1) * 2 + 3;
                    for (int n = 0; n < b[m, 0].sizeX; n++) {
                        result[k, l] = a[n + aM, l+j] * b[i, m][k, n];
                    }
                }
            }
        }
        return result;
    }

    //inverse
    //https://gist.github.com/occar421/feb03a0183e69ecc0189
    public static SparseCovarianceMatrix operator !(CovarianceMatrix a) {
        SparseCovarianceMatrix result = new SparseCovarianceMatrix();
        result.Enlarge(a.count+1);
        for (int i = 0; i < a.count - 1; i++) {
            for (int j = i + 1; j < a.count - 1; j++) {
                Matrix s = a[j, i] * !a[i, i];//Does this work or do we have to consider the matrix as a whole and dont invert submatrices / use s as a float?(TODO)
                for (int k = i; k < a.count; k++) {
                    a[j, k] -= a[i, k] * s;
                }
                for (int k = 0; k <a.count; k++) {
                    result[j, k] -= result[i, k] * s;
                }
            }
        }
        for (int i = a.count - 1; i >= 0; i--) {
            result[i, i] *= !a[i, i];
            a[i, i] *= !a[i, i];
            for (int j = i - 1; j >= 0; j--) {
                Matrix s = a[j, i] * !a[i, i];
                a[j, i] -= s;
                for (int k = 0; k < a.count; k++) {
                    result[j, k] -= result[i, k] * s;
                }
            }
        }
        return result;
    }
}

[Serializable]
public class SparseColumn {

    public IntMatrixDictionary val = new IntMatrixDictionary();
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
        foreach(KeyValuePair<int, Matrix> pair in addition.val) {
            val[pair.Key] += pair.Value;
        }
    }

    public static SparseColumn operator +(SparseColumn a, SparseColumn b) {
        //if (a.size != b.size) throw new MatrixSizeException(a.size, b.size, "+");
        SparseColumn result = new SparseColumn();
        foreach(KeyValuePair<int,Matrix> pair in a.val) {
            result.val.Add(pair.Key, pair.Value + b.val[pair.Key]);
        }
        foreach (KeyValuePair<int, Matrix> pair in b.val) {
            if(result.val[pair.Key] == null) result.val.Add(pair.Key, pair.Value);
        }
        return result;
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
        SparseColumn result = new SparseColumn();//TODO:ignore size of SparseColumn
        foreach(SparseColumn col in a.val) {
            foreach (KeyValuePair<int, Matrix> pair in col.val) {
                IFeature f = b[pair.Key];
                if (f.IsFeature()) {
                    result[pair.Key] += pair.Value * ((Feature) f).feature;
                } else {
                    result[pair.Key] += pair.Value * ((RobotPose) f).pose;
                }
            }
        }
        return result;
    }

    public static SparseColumn operator *(SparseMatrix a, SparseColumn b) {
        if (a == null || b == null) return null;
        SparseColumn result = new SparseColumn();//TODO:ignore size of SparseColumn
        foreach (SparseColumn col in a.val) {
            foreach (KeyValuePair<int, Matrix> pair in col.val) {
                result[pair.Key] += pair.Value * b.val[pair.Key];
            }
        }
        return result;
    }

    public static SparseMatrix operator *(SparseMatrix a, SparseMatrix b) {
        if (a == null || b == null) return null;
        if (a.ColumnCount() != b.RowCount()) throw new MatrixSizeException(a.ColumnCount(), b.RowCount(), "*");
        SparseMatrix result = new SparseMatrix();
        for (int j = 0; j < b.ColumnCount(); j++) result.AddColumn(new SparseColumn());
        int count = a.RowCount();
        result.SetRowCount(count);
        int i = 0;
        foreach(SparseColumn colB in b.val) {
            for(int j = 0; j < count; j++) {
                Matrix resultField = null;
                foreach (KeyValuePair<int, Matrix> pair in colB.val) {
                        resultField += a[j, pair.Key] * pair.Value;
                }
                if (resultField != null) result[j, i] = resultField;
            }
            i++;
        }
        return result;
    }

    //translate
    public static SparseTranslatedMatrix operator ~(SparseMatrix a) {
        return new SparseTranslatedMatrix(a);
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
}

public interface ISparseCovarianceMatrix {
    Matrix this[int i, int j] { get; }
}

[Serializable]
public class SparseCovarianceMatrix : ISparseCovarianceMatrix {
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
    public void Trim(HashSet<int> keepRows, int size) {
        foreach (SparseColumn col in val) {
            var enumerator = keepRows.GetEnumerator();
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
            if (col[size - 1] != null) {
                col[j] = col[size - 1];
                col.Remove(size - 1);
            }
            enumerator.Dispose();
        }
    }

    public void Addition(SparseMatrix addition) {
        if (ColumnCount() != addition.ColumnCount()) throw new MatrixSizeException(ColumnCount(), addition.ColumnCount(), "a");
        int i = 0;
        foreach (SparseColumn colAdd in addition.val) {
            SparseColumn col = val[i];
            foreach (KeyValuePair<int, Matrix> pair in colAdd.val) {
                col[pair.Key] = pair.Value;
            }
            i++;
        }
    }

    public static SparseMatrix operator *(SparseTranslatedMatrix a, SparseCovarianceMatrix b) {
        if (a == null || b == null) return null;
        if (a.ColumnCount() != b.ColumnCount()) throw new MatrixSizeException(a.ColumnCount(), b.ColumnCount(), "*");
        int count = a.RowCount();
        SparseMatrix result = new SparseMatrix();
        for (int i = 0; i < b.ColumnCount(); i++) result.AddColumn(new SparseColumn());
        result.SetRowCount(a.RowCount());
        for (int i = 0; i < count; i++) {
            SparseColumn row = a.GetRow(i);
            for (int j = 0; j < count; j++) {
                SparseColumn col = b.val[j];
                Matrix resultField = null;
                foreach (KeyValuePair<int, Matrix> pair in row.val) {
                    Matrix m;
                    if (col.val.TryGetValue(pair.Key, out m)) {
                        resultField += pair.Value * m;
                    }
                }
                if (resultField != null) result[j, i] = resultField;
            }
        }
        return result;
    }
}

public class DefaultedSparseCovarianceMatrix : ISparseCovarianceMatrix {

    private SparseCovarianceMatrix m;
    private Matrix def;

    public DefaultedSparseCovarianceMatrix(SparseCovarianceMatrix m, Matrix def) {
        this.m = m;
        this.def = def;
    }

    public Matrix this[int i, int j] {
        get {
            if (i < 0) return def;
            return m[i, j];
        }
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
        for (int i = 0; i < val.Count; i++) {//Rows
            if (this[i, i] != null) {
                Matrix m = rightHandSide[i];
                for (int j = 0; j < i; j++) {//Columns
                    m -= this[j, i] * result[j];
                }
                if (m != null) result[i] = m * !this[i, i];
            }
        }
        return result;
    }

    public SparseColumn solveUpperRightSparse(SparseColumn rightHandSide) {
        SparseColumn result = new SparseColumn();
        for (int i = val.Count - 1; i >= 0; i++) {//Rows
            if (this[i, i] != null) {
                Matrix m = rightHandSide[i];
                for (int j = val.Count - 1; j < i; j++) {//Columns
                    //TODO:If we just switch rows and cols in the matrix here we do not have to translate it.(right?)Could this be made faster by accessing the dictionary-key-enumerator directly with skipping the zeros over the column?
                    m -= this[i, j] * result[j];
                }
                if (m != null) result[i] = m * !this[i, i];
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
        get {
            return val[i][j];
        }
        set {

        }
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

    public void Remove(int i) {
        val.Remove(i);
    }

    public void Clear() {
        val.Clear();
    }
}