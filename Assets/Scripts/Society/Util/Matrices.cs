using Superbest_random;
using System;
using System.Collections.Generic;

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
                    result[i, j] += a[i, k] * b[k, j];
                }
            }
        }
        return result;
    }

    public static Matrix operator *(Matrix a, UnityEngine.Vector3 b) {
        if (a == null || b == null) return null;
        if (a.sizeX != 3) throw new MatrixSizeException(a.sizeX, 3, "*");
        Matrix result = new Matrix(1, a.sizeY);
        for (int j = 0; j < result.sizeY; j++) {
            result[0, j]  = a[j, 0] * b.x + a[j, 1] * b.y + a[j, 2] * b.z;
        }
        return result;
    }

    public static Matrix operator *(Matrix a, UnityEngine.Vector4 b) {
        if (a == null || b == null) return null;
        if (a.sizeX != 2) throw new MatrixSizeException(a.sizeX, 2, "*");
        Matrix result = new Matrix(1, a.sizeY);
        UnityEngine.Vector2 center = Geometry.Center(b);
        for (int j = 0; j < result.sizeY; j++) {
            //TODO: expand all 2D Matrices to 4D or make sure this is okay:
            result[0, j] = a[j, 0] * center.x + a[j, 1] * center.y;
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

    //inverse
    //https://gist.github.com/occar421/feb03a0183e69ecc0189
    public static SparseCovarianceMatrix operator !(CovarianceMatrix a) {
        SparseCovarianceMatrix result = new SparseCovarianceMatrix();
        result.Enlarge3();
        result.Enlarge2(a.count);
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

    public void Addition(SparseColumn addition) {
        //TD
    }

    public static SparseColumn operator +(SparseColumn a, SparseColumn b) {
        if (a.size != b.size) throw new MatrixSizeException(a.size, b.size, "+");
        SparseColumn result = new SparseColumn(a.size);
        foreach(KeyValuePair<int,Matrix> pair in a.val) {
            result.val.Add(pair.Key, pair.Value + b.val[pair.Key]);
        }
        foreach (KeyValuePair<int, Matrix> pair in b.val) {
            if(result.val[pair.Key] == null) result.val.Add(pair.Key, pair.Value);
        }
        return result;
    }
}

/*abstract class SparseBase {
    private List<SparseColumn> val = new List<SparseColumn>();

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    public abstract int RowCount();

    public int ColumnCount() {
        return val.Count;
    }
}*/

public class SparseMatrix {

    internal List<SparseColumn> val = new List<SparseColumn>();
    private int rowCount = 0;

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    public void Enlarge2(int i) {
        while (i-- != 0) val.Add(new SparseColumn(2));
    }

    //Enlarges the sparse covariance matrix by a new row/col for a 3x3 matrix (a robot pose)
    public void Enlarge3() {
        val.Add(new SparseColumn(3));
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
        SparseColumn result = new SparseColumn(2);//TODO:ignore size of SparseColumn
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
        SparseColumn result = new SparseColumn(b.size);//TODO:ignore size of SparseColumn
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
        for (int j = 0; j < b.ColumnCount(); j++) result.AddColumn(new SparseColumn(b.val[j].size));
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

    /*public Matrix this[int i, int j] {
        get { return matrix[j, i]; }
        set { matrix[j, i] = value; }
    }*/

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

public class SparseCovarianceMatrix {
    private List<SparseColumn> val = new List<SparseColumn>();
    private int count = 0;

    public SparseCovarianceMatrix() { }

    public void Enlarge2(int i) {
        count += i * 2;
        while (i-- != 0) val.Add(new SparseColumn(2));
    }

    //Enlarges the sparse covariance matrix by a new row/col for a 3x3 matrix (a robot pose)
    public void Enlarge3() {
        val.Add(new SparseColumn(3));
        count += 3;
    }

    public Matrix this[int i, int j] {
        get { return val[i][j]; }
        set { val[i][j] = value; }
    }

    public SparseColumn GetRow(int i) {
        throw new NotImplementedException();
    }

    public SparseColumn GetColumn(int i) {
        return val[i];
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

    internal void Clear() {
        foreach (SparseColumn col in val) col.Clear();
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
        for (int i = 0; i < b.ColumnCount(); i++) result.AddColumn(new SparseColumn(b.val[i].size));
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