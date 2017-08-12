using ev3devMapping.Society;
using NUnit.Framework;
using UnityEngine;

/*
  To test the operators we have to assign their result to a variable,
  even when we expect them to throw an exception.
  Therefore this warning about unused variables is disabled for all tmp variables in this file.
*/
#pragma warning disable 0219

namespace ev3devMapping.Testing {
    public class TestingMatrices: ITesting {

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestValues() {
            Matrix m = new Matrix(2),
                   n = new Matrix(2, 2);
            Assert.IsTrue(m.sizeX == m.sizeY && m.sizeX == 2);
            Assert.AreEqual(1.0d, m[0, 0], DELTA);
            Assert.AreEqual(1.0d, m[1, 1], DELTA);
            Assert.AreEqual(0.0d, m[0, 1], DELTA);
            Assert.AreEqual(0.0d, m[1, 0], DELTA);
            Assert.IsFalse(m.IsEmpty());
            m.Empty();
            Assert.IsTrue(m.IsEmpty());
            Assert.IsTrue(n.IsEmpty());

            m = new Matrix(3);
            Assert.IsTrue(m.sizeX == m.sizeY && m.sizeX == 3);
            Assert.AreEqual(1.0d, m[0, 0], DELTA);
            Assert.AreEqual(1.0d, m[1, 1], DELTA);
            Assert.AreEqual(1.0d, m[2, 2], DELTA);
            Assert.AreEqual(0.0d, m[0, 1], DELTA);
            Assert.AreEqual(0.0d, m[0, 2], DELTA);
            Assert.AreEqual(0.0d, m[1, 0], DELTA);
            Assert.AreEqual(0.0d, m[1, 2], DELTA);
            Assert.AreEqual(0.0d, m[2, 0], DELTA);
            Assert.AreEqual(0.0d, m[2, 1], DELTA);
            Assert.IsFalse(m.IsEmpty());
            m.Empty();
            Assert.IsTrue(m.IsEmpty());

            m = new Matrix(2, 3);
            Assert.IsTrue(m.sizeX == 2 && m.sizeY == 3);
            Assert.AreEqual(0.0d, m[0, 0], DELTA);
            Assert.AreEqual(0.0d, m[0, 1], DELTA);
            Assert.AreEqual(0.0d, m[0, 2], DELTA);
            Assert.AreEqual(0.0d, m[1, 0], DELTA);
            Assert.AreEqual(0.0d, m[1, 1], DELTA);
            Assert.AreEqual(0.0d, m[1, 2], DELTA);
            Assert.IsTrue(m.IsEmpty());
            m[0, 0] = 12345678.0f;
            m[0, 1] = 98765432.0f;
            m[0, 2] = -12345678.0f;
            m[1, 1] = -98765432.0f;
            Assert.IsFalse(m.IsEmpty());

            n = m.Duplicate();
            Assert.AreNotSame(m, n);
            Assert.IsFalse(n.IsEmpty());
            m.Empty();
            Assert.IsTrue(m.IsEmpty());

            Assert.IsFalse(n.IsEmpty());
            Assert.AreEqual(12345678.0d, n[0, 0], DELTA);
            Assert.AreEqual(98765432.0d, n[0, 1], DELTA);
            Assert.AreEqual(-12345678.0d, n[0, 2], DELTA);
            Assert.AreEqual(-98765432.0d, n[1, 1], DELTA);
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestMatrixSubstraction() {
            Matrix m = new Matrix(2),
                   n = new Matrix(2, 2),
                   o = new Matrix(2, 2);
            m[0, 1] = 3.5f;
            n[0, 0] = -5.0f;
            n[0, 1] = 123.0f;
            n[1, 0] = 7.0f;
            o[0, 0] = -4.0f;
            o[0, 1] = 126.5f;
            o[1, 0] = 7.0f;
            o[1, 1] = 1.0f;
            m = m + n;
            o[0, 0] = - o[0, 0];
            o[0, 1] = - o[0, 1];
            o[1, 0] = - o[1, 0];
            o[1, 1] = - o[1, 1];
            Assert.AreEqual(o, -m);
            Assert.AreEqual(m, -(-m));
            Assert.AreNotEqual(m, -m);
            Assert.AreEqual(m, -(-m));
            n.Empty();
            Assert.AreEqual(n, -n);
            Assert.AreEqual(null, -((Matrix) null));
            
            //Substraction
            m = new Matrix(2);
            Matrix negative = new Matrix(2, 2);
            negative[0, 0] = -1.0f;
            negative[1, 1] = -1.0f;
            Assert.AreEqual(m, m - (Matrix) null);
            Assert.AreEqual(negative, (Matrix) null - m);
            Assert.AreEqual(n, n - (Matrix) null);
            Assert.AreEqual(n, (Matrix) null - n);
            Assert.AreEqual(m, m - n);
            Assert.AreEqual(negative, n - m);
            Assert.AreEqual(n, n - n);
            m[0, 1] = 3.5f;
            n[0, 0] = -5.0f;
            n[0, 1] = 123.0f;
            n[1, 0] = 7.0f;

            o[0, 0] = 6.0f;
            o[0, 1] = -119.5f;
            o[1, 0] = -7.0f;
            o[1, 1] = 1.0f;
            Assert.AreEqual(o, m - n);
            Assert.AreNotEqual(o, n - m);
            n.Empty();
            Assert.AreEqual(n,o - o);

            //Exception
            n = new Matrix(2, 3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m - n; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = n - m; });
            n = new Matrix(3, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m - n; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = n - m; });
            n = new Matrix(3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m - n; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = n - m; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestMatrixAddition() {
            Matrix m = new Matrix(2),
                   n = new Matrix(2, 2),
                   o = new Matrix(2, 2);
            Assert.AreEqual(m, m + (Matrix) null);
            Assert.AreEqual(m, (Matrix) null + m);
            Assert.AreEqual(n, n + (Matrix) null);
            Assert.AreEqual(n, (Matrix) null + n);
            Assert.AreEqual(m, m + n);
            Assert.AreEqual(m, n + m);
            Assert.AreEqual(n, n + n);
            m[0, 1] = 3.5f;
            n[0, 0] = -5.0f;
            n[0, 1] = 123.0f;
            n[1, 0] = 7.0f;
            o[0, 0] = -4.0f;
            o[0, 1] = 126.5f;
            o[1, 0] = 7.0f;
            o[1, 1] = 1.0f;
            Assert.AreEqual(o, m + n);
            Assert.AreEqual(o, n + m);
            
            n = new Matrix(2, 3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m + n; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = n + m; });
            n = new Matrix(3, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m + n; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = n + m; });
            n = new Matrix(3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m + n; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = n + m; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestMatrixInverse() {
            Matrix m = new Matrix(2),
                   n = new Matrix(2, 2),
                   o = new Matrix(2);
            Assert.AreEqual(m, !m);

            //Inverse of an empty Matrix is undefined. Returning Pseudoinverse:
            Assert.AreEqual(null, !((Matrix) null));
            Assert.AreEqual(null, !n);
            
            n[0, 0] = 3.0f;
            n[0, 1] = 5.0f;
            n[1, 0] = 10.0f;
            n[1, 1] = 7.0f;

            o[0, 0] = -7.0f / 29.0f;
            o[0, 1] = 5.0f / 29.0f;
            o[1, 0] = 10.0f / 29.0f;
            o[1, 1] = -3.0f / 29.0f;

            Assert.AreEqual(o, !n);
            Assert.AreEqual(n, !o);
            Assert.AreEqual(m, o * !o);
            Assert.AreEqual(m, n * !n);

            n = new Matrix(2, 3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = !n; });
            n = new Matrix(3, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = !n; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestMatrixTranslate() {
            Matrix m = new Matrix(3),
                   n = new Matrix(2, 3),
                   o = ~m;
            Assert.AreEqual(null, ~((Matrix) null));
            Assert.IsFalse(o.IsEmpty());
            Assert.AreEqual(m, o);
            o = ~n;
            Assert.AreNotEqual(n, o);
            Assert.IsTrue(o.IsEmpty());
            Assert.AreEqual(n.sizeX, o.sizeY);
            Assert.AreEqual(n.sizeY, o.sizeX);
            n[0, 0] = 1.0f;
            n[0, 1] = 2.0f;
            n[0, 2] = 3.0f;
            n[1, 0] = 4.0f;
            n[1, 1] = 5.0f;
            n[1, 2] = 6.0f;
            o[0, 0] = 1.0f;
            o[1, 0] = 2.0f;
            o[2, 0] = 3.0f;
            o[0, 1] = 4.0f;
            o[1, 1] = 5.0f;
            o[2, 1] = 6.0f;
            Assert.AreEqual(n, ~o);
            Assert.AreEqual(o, ~n);
            Assert.AreEqual(n, ~(~n));
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestMatrixMultiplication() {
            Matrix m = new Matrix(3),
                   n = new Matrix(3, 3),
                   o = new Matrix(3, 3);
            Assert.AreEqual(null, m * (Matrix) null);
            Assert.AreEqual(null, (Matrix) null * m);
            Assert.AreEqual(null, ((Matrix) null) * ((Matrix) null));
            Assert.AreEqual(m, m * m);
            Assert.AreEqual(n, n * m);
            Assert.AreEqual(n, n * n);
            Assert.AreEqual(n, m * n);
            m[0, 0] = 2.0f;
            m[1, 1] = 3.0f;
            m[2, 2] = 4.0f;
            n[0, 0] = 4.0f;
            n[1, 1] = 9.0f;
            n[2, 2] = 16.0f;
            Assert.AreEqual(n, m * m);
            m[1, 0] = 7.0f;
            m[2, 0] = 10.0f;
            m[0, 1] = 5.0f;
            m[2, 1] = 11.0f;
            m[0, 2] = 6.0f;
            m[1, 2] = 8.0f;

            n[1, 0] = 14.0f;
            n[2, 0] = 17.0f;
            n[0, 1] = 12.0f;
            n[2, 1] = 18.0f;
            n[0, 2] = 13.0f;
            n[1, 2] = 15.0f;

            o[0, 0] = 222.0f;
            o[1, 0] = 241.0f;
            o[2, 0] = 320.0f;
            o[0, 1] = 199.0f;
            o[1, 1] = 262.0f;
            o[2, 1] = 315.0f;
            o[0, 2] = 172.0f;
            o[1, 2] = 216.0f;
            o[2, 2] = 310.0f;
            Assert.AreEqual(o, m * n);
            Assert.AreNotEqual(o, n * m);

            n = new Matrix(2, 3);
            n[0, 0] = 12.0f;
            n[1, 0] = 13.0f;
            n[0, 1] = 14.0f;
            n[1, 1] = 15.0f;
            n[0, 2] = 16.0f;
            n[1, 2] = 17.0f;

            o = new Matrix(2, 3);
            o[0, 0] = 282.0f;
            o[1, 0] = 301.0f;
            o[0, 1] = 278.0f;
            o[1, 1] = 297.0f;
            o[0, 2] = 248.0f;
            o[1, 2] = 266.0f;
            Assert.AreEqual(o, m * n);
            Assert.Throws<MatrixSizeException>(() => { var tmp = n * m; });

            n = new Matrix(3, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
            Assert.DoesNotThrow(() => { var tmp = n * m; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestVector3Multiplication() {
            Matrix m = new Matrix(3),
                   o = new Matrix(1, 3);
            Vector3 v = new Vector3(0.0f, 0.0f, 0.0f);
            Assert.AreEqual(null, (Matrix) null * v);
            Assert.AreEqual(o, m * v);
            m.Empty();
            Assert.AreEqual(o, m * v);
            v = new Vector3(1.0f, 1.0f, 1.0f);
            m[0, 0] = 2.0f;
            m[1, 1] = 3.0f;
            m[2, 2] = 4.0f;

            o[0, 0] = 2.0f;
            o[0, 1] = 3.0f;
            o[0, 2] = 4.0f;
            Assert.AreEqual(o, m * v);
            v = new Vector3(-8.0f, 10.0f, 6.0f);
            o[0, 0] = -16.0f;
            o[0, 1] = 30.0f;
            o[0, 2] = 24.0f;
            Assert.AreEqual(o, m * v);
            m[1, 0] = 7.0f;
            m[2, 0] = 10.0f;
            m[0, 1] = 5.0f;
            m[2, 1] = 11.0f;
            m[0, 2] = 6.0f;
            m[1, 2] = 8.0f;

            o[0, 0] = 114.0f;
            o[0, 1] = 56.0f;
            o[0, 2] = 56.0f;
            Assert.AreEqual(o, m * v);
            m = new Matrix(4, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * v; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestVector2Multiplication() {
            Matrix m = new Matrix(2),
                   o = new Matrix(1, 2);
            Assert.AreEqual(null, (Matrix) null * Vector2.zero);
            Assert.AreEqual(null, Vector2.zero * (Matrix) null );
            Assert.AreEqual(o, m * Vector2.zero);
            Assert.AreEqual(~o, Vector2.zero * m);
            m.Empty();
            Assert.AreEqual(o, m * Vector2.zero);
            Assert.AreEqual(~o, Vector2.zero * m);
            Vector2 v = new Vector2(1.0f, 1.0f);
            m[0, 0] = 2.0f;
            m[1, 1] = 3.0f;
            o[0, 0] = 2.0f;
            o[0, 1] = 3.0f;
            Assert.AreEqual(o, m * v);
            Assert.AreEqual(~o, v * m);
            v = new Vector2(-8.0f, 10.0f);
            o[0, 0] = -16.0f;
            o[0, 1] = 30.0f;
            Assert.AreEqual(o, m * v);
            Assert.AreEqual(~o, v * m);
            m[1, 0] = 7.0f;
            m[0, 1] = 5.0f;
            o[0, 0] = 54.0f;
            o[0, 1] = -10.0f;
            Assert.AreEqual(o, m * v);
            o[0, 0] = 34;
            o[0, 1] = -26.0f;
            Assert.AreEqual(~o, v * m);
            m = new Matrix(3, 3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * v; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = v * m; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestDiagonalMatrixAddition() {
            Matrix m = new Matrix(2),
                   o = new Matrix(2, 2);
            DiagonalMatrix d = new DiagonalMatrix();
            d.Enlarge(2);
            Assert.AreEqual(m, m + (DiagonalMatrix) null);
            Assert.AreEqual(m, (Matrix) null + d);
            Assert.AreEqual(o, o + (DiagonalMatrix) null);
            Assert.AreEqual(m, o + d);
            d[0] = 0.0f;
            d[1] = 0.0f;
            Assert.AreEqual(o, o + d);
            Assert.AreEqual(m, m + d);
            d[0] = -9.0f;
            d[1] = 17.0f;

            m[0, 0] = 34.0f;
            m[1, 0] = 16.0f;
            m[0, 1] = -45.0f;
            m[1, 1] = 99.0f;

            o[0, 0] = 25.0f;
            o[1, 0] = 16.0f;
            o[0, 1] = -45.0f;
            o[1, 1] = 116.0f;
            Assert.AreEqual(o, m + d);
            
            m = new Matrix(2, 3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m + d; });
            m = new Matrix(3, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m + d; });
            m = new Matrix(3);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m + d; });
        }

        [Test]
        [TestOf(typeof(Matrix))]
        public void MatrixTestDiagonalMatrixMultiplication() {
            Matrix m = new Matrix(3),
                   o = new Matrix(3, 3);
            DiagonalMatrix d = new DiagonalMatrix();
            d.Enlarge(3);
            Assert.AreEqual(null, m * (DiagonalMatrix) null);
            Assert.AreEqual(null, (Matrix) null * d);
            Assert.AreEqual(null, ((Matrix) null) * ((DiagonalMatrix) null));
            Assert.AreEqual(m, m * d);
            Assert.AreEqual(o, o * d);
            o[0, 0] = 2.0f;
            o[1, 1] = 3.0f;
            o[2, 2] = 4.0f;
            Assert.AreEqual(o, o * d);
            d[0] = 2.0f;
            d[1] = 3.0f;
            d[2] = 4.0f;
            Assert.AreEqual(o, m * d);

            m[0, 0] = 2.0f;
            m[1, 1] = 3.0f;
            m[2, 2] = 4.0f;
            m[1, 0] = 7.0f;
            m[2, 0] = 10.0f;
            m[0, 1] = 5.0f;
            m[2, 1] = 11.0f;
            m[0, 2] = 6.0f;
            m[1, 2] = 8.0f;

            d[0] = 14.0f;
            d[1] = 17.0f;
            d[2] = 12.0f;

            o[0, 0] = 28.0f;
            o[1, 0] = 119.0f;
            o[2, 0] = 120.0f;
            o[0, 1] = 70.0f;
            o[1, 1] = 51.0f;
            o[2, 1] = 132.0f;
            o[0, 2] = 84.0f;
            o[1, 2] = 136.0f;
            o[2, 2] = 48.0f;
            Assert.AreEqual(o, m * d);

            m = new Matrix(3, 2);
            m[0, 0] = 2.0f;
            m[1, 1] = 3.0f;
            m[0, 1] = 5.0f;
            m[2, 0] = 10.0f;
            m[1, 0] = 7.0f;
            m[2, 1] = 11.0f;

            o = new Matrix(3, 2);
            o[0, 0] = 28.0f;
            o[0, 1] = 70.0f;
            o[1, 0] = 119.0f;
            o[1, 1] = 51.0f;
            o[2, 0] = 120.0f;
            o[2, 1] = 132.0f;
            Assert.AreEqual(o, m * d);
            m = new Matrix(2, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * d; });
        }
    }
}

#pragma warning restore 0219