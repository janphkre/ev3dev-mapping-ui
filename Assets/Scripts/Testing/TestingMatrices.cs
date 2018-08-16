using ev3devMapping.Society;
using NUnit.Framework;
using System.Collections.Generic;
using UnityEngine;

/*
  To test the operators we have to assign their result to a variable,
  even when we expect them to throw an exception.
  Therefore this warning about unused variables is disabled for all tmp variables in this file.
*/
#pragma warning disable 0219

namespace ev3devMapping.Testing {
    public class TestingMatrices: ITesting {

    #region Matrix
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
            Matrix m = new Matrix(2, 2),
                   n = new Matrix(2, 2),
                   o = new Matrix(2, 2);
            //Inverse of an empty Matrix:
            Assert.AreEqual(null, !((Matrix) null));
            Assert.AreEqual(null, !n);
            //Inverse:
            m[0, 0] = 3.0f;
            m[1, 0] = 10.0f;
            m[0, 1] = 5.0f;
            m[1, 1] = 7.0f;

            n[0, 0] = 3.0f;
            n[1, 0] = 10.0f;
            n[0, 1] = 5.0f;
            n[1, 1] = 7.0f;

            o[0, 0] = -7.0f / 29.0f;
            o[1, 0] = 10.0f / 29.0f;
            o[0, 1] = 5.0f / 29.0f;
            o[1, 1] = -3.0f / 29.0f;

            Matrix p = !n;
            Assert.AreEqual(o, p);
            p = !o;
            Assert.AreEqual(m, p);
            m = new Matrix(2);
            Assert.AreEqual(m, n);
            Assert.AreEqual(m, o);
            m = new Matrix(3);
            n = new Matrix(3);
            o = new Matrix(3);

            m[0, 0] = 6f;
            m[1, 0] = 0f;
            m[2, 0] = 0f;
            m[0, 1] = 11f;
            m[1, 1] = 12f;
            m[2, 1] = 0f;
            m[0, 2] = 16f;
            m[1, 2] = 17f;
            m[2, 2] = 18f;

            n[0, 0] = 6f;
            n[1, 0] = 0f;
            n[2, 0] = 0f;
            n[0, 1] = 11f;
            n[1, 1] = 12f;
            n[2, 1] = 0f;
            n[0, 2] = 16f;
            n[1, 2] = 17f;
            n[2, 2] = 18f;

            o[0, 0] = 1f / 6f;
            o[1, 0] = 0f;
            o[2, 0] = 0f;
            o[0, 1] = -11f / 72f;
            o[1, 1] = 1f / 12f;
            o[2, 1] = 0f;
            o[0, 2] = -5f / 1296f;
            o[1, 2] = -17f / 216f;
            o[2, 2] = 1f / 18f;

            p = !n;
            Assert.AreEqual(o, p);
            p = !o;
            Assert.AreEqual(m, p);
            //Inverse of identity:
            m = new Matrix(2);
            p = !m;
            m = new Matrix(2);
            Assert.AreEqual(m, p);

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

    #endregion 
    #region DiagonalMatrix

        [Test]
        [TestOf(typeof(DiagonalMatrix))]
        public void DiagonalMatrixTestValues() {
            DiagonalMatrix m = new DiagonalMatrix();
            Matrix o = new Matrix(0, 0);
            Assert.AreEqual(o, m.ToMatrix());
            Assert.IsTrue(m.Size() == 0);
            m.Enlarge(RANDOM_ITERATIONS);
            Assert.IsTrue(m.Size() == RANDOM_ITERATIONS);
            for(int i = 0; i < RANDOM_ITERATIONS; i++) Assert.AreEqual(1.0d, m[i], DELTA);
            Matrix n = m.ToMatrix();
            o = new Matrix(RANDOM_ITERATIONS);
            Assert.AreEqual(o, n);
            m[0] = 12345678.0f;
            m[1] = 98765432.0f;
            m[2] = -12345678.0f;
            m[55] = -98765432.0f;
            Assert.AreNotSame(m, n);
            Assert.AreNotEqual(12345678.0d, n[0, 0]);
            Assert.AreNotEqual(98765432.0d, n[1, 1]);
            Assert.AreNotEqual(-12345678.0d, n[2, 2]);
            Assert.AreNotEqual(-98765432.0d, n[55, 55]);
            n = m.ToMatrix();
            Assert.AreEqual(12345678.0d, n[0, 0], DELTA);
            Assert.AreEqual(98765432.0d, n[1, 1], DELTA);
            Assert.AreEqual(-12345678.0d, n[2, 2], DELTA);
            Assert.AreEqual(-98765432.0d, n[55, 55], DELTA);
            n.Empty();
            Assert.AreEqual(12345678.0d, m[0], DELTA);
            Assert.AreEqual(98765432.0d, m[1], DELTA);
            Assert.AreEqual(-12345678.0d, m[2], DELTA);
            Assert.AreEqual(-98765432.0d, m[55], DELTA);
        }

        [Test]
        [TestOf(typeof(DiagonalMatrix))]
        public void DiagonalMatrixTestAddition() {
            DiagonalMatrix m = new DiagonalMatrix(),
                           n = new DiagonalMatrix();
            Matrix o = new Matrix(0, 0);
            Assert.AreEqual(o, (m + n).ToMatrix());
            Assert.AreEqual(o, (m + null).ToMatrix());
            Assert.AreEqual(o, (null + n).ToMatrix());
            m.Enlarge(10);
            n.Enlarge(10);
            o = new Matrix(10);
            Assert.AreEqual(o, (m + null).ToMatrix());
            Assert.AreEqual(o, (null + n).ToMatrix());
            for(int i = 0; i < 10; i++) o[i, i] = 2.0f;
            Assert.AreEqual(o, (m + n).ToMatrix());
            m[0] = 100.0f;
            m[1] = 200.0f;
            m[2] = 0.0f;
            m[3] = -50.0f;
            /*m[4] = 1.0f*/
            m[5] = -243.0f;
            m[6] = -666.0f;
            m[7] = 567.0f;
            m[8] = 888.0f;
            m[9] = 10.0f;

            n[0] = 100.0f;
            n[1] = 401.0f;
            n[2] = 33.0f;
            n[3] = 0.0f;
            n[4] = -23.0f;
            /*m[5] = 1.0f*/
            n[6] = 22.0f;
            n[7] = 5678.0f;
            n[8] = -788.0f;
            n[9] = 50.0f;

            o[0, 0] = 200.0f;
            o[1, 1] = 601.0f;
            o[2, 2] = 33.0f;
            o[3, 3] = -50.0f;
            o[4, 4] = -22.0f;
            o[5, 5] = -242.0f;
            o[6, 6] = -644.0f;
            o[7, 7] = 6245.0f;
            o[8, 8] = 100.0f;
            o[9, 9] = 60.0f;
            Assert.AreEqual(o, (m + n).ToMatrix());

            n.Enlarge(5);
            Assert.Throws<MatrixSizeException>(() => { var tmp = n + m; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = m+ n; });
        }

        [Test]
        [TestOf(typeof(DiagonalMatrix))]
        public void DiagonalMatrixTestMultiplication() {
            DiagonalMatrix m = new DiagonalMatrix(),
                           n = new DiagonalMatrix();
            Matrix o = new Matrix(0, 0);
            Assert.AreEqual(o, (m * n).ToMatrix());
            Assert.AreEqual(null, m * (DiagonalMatrix) null);
            Assert.AreEqual(null, (DiagonalMatrix) null * n);
            m.Enlarge(10);
            n.Enlarge(10);
            o = new Matrix(10);
            Assert.AreEqual(o, (m * n).ToMatrix());
            Assert.AreEqual(null, m * (DiagonalMatrix) null);
            Assert.AreEqual(null, (DiagonalMatrix) null * n);
            Assert.AreEqual(null, ((DiagonalMatrix) null) * ((DiagonalMatrix) null));

            m[0] = 10.0f;
            m[1] = 20.0f;
            m[2] = 0.0f;
            m[3] = -50.0f;
            /*m[4]*/
            m[5] = -24.0f;
            m[6] = -66.0f;
            m[7] = 56.0f;
            m[8] = 88.0f;
            m[9] = 10.0f;

            n[0] = 10.0f;
            n[1] = 41.0f;
            n[2] = 32.0f;
            n[3] = 0.0f;
            n[4] = -23.0f;
            /*m[5]*/
            n[6] = 22.0f;
            n[7] = 56.0f;
            n[8] = -78.0f;
            n[9] = 50.0f;

            o[0, 0] = 100.0f;
            o[1, 1] = 820.0f;
            o[2, 2] = 0.0f;
            o[3, 3] = 0.0f;
            o[4, 4] = -23.0f;
            o[5, 5] = -24.0f;
            o[6, 6] = -1452.0f;
            o[7, 7] = 3136.0f;
            o[8, 8] = -6864.0f;
            o[9, 9] = 500.0f;

            Assert.AreEqual(o, (m * n).ToMatrix());
            Assert.AreEqual(o, (n * m).ToMatrix());

            n.Enlarge(5);
            Assert.Throws<MatrixSizeException>(() => { var tmp = n * m; });
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
        }

        [Test]
        [TestOf(typeof(DiagonalMatrix))]
        public void DiagonalMatrixTestTranslate() {
            DiagonalMatrix m = new DiagonalMatrix();
            Matrix o = new Matrix(0, 0);
            Assert.AreEqual(o, (~m).ToMatrix());
            Assert.AreEqual(m, ~m);
            Assert.AreEqual(null, ~((DiagonalMatrix) null));
            m.Enlarge(10);
            Assert.AreEqual(m, ~m);
            m[0] = 10.0f;
            m[1] = 20.0f;
            m[2] = 0.0f;
            m[3] = -50.0f;
            /*m[4]*/
            m[5] = -24.0f;
            m[6] = -66.0f;
            m[7] = 56.0f;
            m[8] = 88.0f;
            m[9] = 10.0f;
            Assert.AreEqual(m, ~m);
        }
        #endregion
    #region Row
        [Test]
        [TestOf(typeof(Row))]
        public void RowTestValues() {
            Row r = new Row(1, 3);
            Assert.IsTrue(r.sizeY == 3);
            Assert.IsTrue(r.val.Count == 1);
            Assert.IsTrue(r.val[0].sizeX == r.val[0].sizeX && r.val[0].sizeX == 3);

            r.Enlarge(10);
            Assert.IsTrue(r.sizeY == 3);
            Assert.IsTrue(r.val.Count == 11);
            Assert.IsTrue(r.val[0].sizeX == r.val[0].sizeX && r.val[0].sizeX == 3);
            for(int i = 1; i < 11; i++) Assert.IsTrue(r.val[i].sizeX == 2 && r.val[i].sizeY == 3);

            r = new Row(11);
            Assert.IsTrue(r.sizeY == 2);
            Assert.IsTrue(r.val.Count == 11);
            Assert.IsTrue(r.val[0].sizeX == 3 && r.val[0].sizeY == 2);
            for(int i = 1; i < 11; i++) Assert.IsTrue(r.val[i].sizeX == 2 && r.val[i].sizeY == 2);

            r.Enlarge(10);
            Assert.IsTrue(r.val.Count == 21);
            Assert.IsTrue(r.val[0].sizeX == 3 && r.val[0].sizeY == 2);
            for(int i = 1; i < 21; i++) Assert.IsTrue(r.val[i].sizeX == 2 && r.val[i].sizeY == 2);
        }
    #endregion
    #region CovarianceMatrix
        [Test]
        [TestOf(typeof(CovarianceMatrix))]
        public void CovarianceMatrixTestValues() {
            CovarianceMatrix m = new CovarianceMatrix(10);
            Assert.IsTrue(m.count == 21);
            Assert.IsTrue(m.val.Count == 10);
            Assert.IsTrue(m.val[0].sizeY == 3);
            for(int i = 1; i < 10; i++) Assert.IsTrue(m.val[i].sizeY == 2 && m.val[i].val.Count == 10);
            m.Enlarge(10);
            Assert.IsTrue(m.count == 41);
            Assert.IsTrue(m.val.Count == 20);
            Assert.IsTrue(m.val[0].sizeY == 3);
            for(int i = 1; i < 20; i++) Assert.IsTrue(m.val[i].sizeY == 2 && m.val[i].val.Count == 20);
            Matrix n = new Matrix(3, 3);
            Assert.AreEqual(n,m[0, 0]);
            n = new Matrix(2, 3);
            for(int i = 1; i < 20; i++) Assert.AreEqual(n, m[i, 0]);
            n = new Matrix(3, 2);
            for(int i = 1; i < 20; i++) Assert.AreEqual(n, m[0, i]);
            n = new Matrix(2, 2);
            for(int i = 1; i < 20; i++) {
                for(int j = 1; j < 20; j++) Assert.AreEqual(n, m[i, j]);
            }
            m = new CovarianceMatrix(new System.Random());
            Assert.IsTrue(m.count == 3);
            Assert.IsTrue(m.val.Count == 1);
            Assert.IsTrue(m.val[0].sizeY == 3);
            m.Enlarge(10);
            Assert.IsTrue(m.count == 23);
            Assert.IsTrue(m.val.Count == 11);
            Assert.IsTrue(m.val[0].sizeY == 3);
            for(int i = 1; i < 11; i++) Assert.IsTrue(m.val[i].sizeY == 2 && m.val[i].val.Count == 11);
            m = new CovarianceMatrix(new System.Random(), 10);
            Assert.IsTrue(m.count == 21);
            Assert.IsTrue(m.val.Count == 10);
            Assert.IsTrue(m.val[0].sizeY == 3);
            m.Enlarge(10);
            Assert.IsTrue(m.count == 41);
            Assert.IsTrue(m.val.Count == 20);
            Assert.IsTrue(m.val[0].sizeY == 3);
            for(int i = 10; i < 20; i++) Assert.IsTrue(m.val[i].sizeY == 2 && m.val[i].val.Count == 20);
            Assert.AreEqual(m, m.Duplicate());
            m = new CovarianceMatrix(5, 1.0f);
            Assert.AreEqual(new Matrix(11), m);
        }

        [Test]
        [TestOf(typeof(CovarianceMatrix))]
        public void CovarianceMatrixTestMultiplication() {
            CovarianceMatrix m = new CovarianceMatrix(10);
            Matrix n = new Matrix(21, 21);
            Assert.AreEqual(n, (n * m).ToMatrix());
            Assert.AreEqual(null, ((Matrix) null) * m);
            Assert.AreEqual(null, n * ((CovarianceMatrix) null));
            Assert.AreEqual(null, ((Matrix) null) * ((CovarianceMatrix) null));
            n = new Matrix(3, 21);
            Assert.AreEqual(n, m * n);
            Assert.AreEqual(null, m * ((Matrix) null));
            Assert.AreEqual(null, ((CovarianceMatrix) null) * n);
            Assert.AreEqual(null, ((CovarianceMatrix) null) * ((Matrix) null));
            m[0, 0] = new Matrix(3);
            int i;
            for(i = 1; i < 10; i++) m[i, i] = new Matrix(2);
            Assert.AreEqual(n, m * n);
            for(i = 0; i < 21; i++) {
                n[0, i] = i+1;
                n[1, i] = i+21;
                n[2, i] = i+42;
            }
            Assert.AreEqual(n, m * n);
            m = new CovarianceMatrix(2);
            i = 0;
            int j = 0,
                k = 0,
                l = 0;
            while(true) {
                m[i, j][k, l] = 11 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            n = new Matrix(2, 5);
            for(i = 0; i < 5; i++) {
                n[0, i] = i*2+1;
                n[1, i] = i*2+2;
            }
            Matrix o = new Matrix(2, 5);
            o[0, 0] = 345;
            o[1, 0] = 410;
            o[0, 1] = 470;
            o[1, 1] = 560;
            o[0, 2] = 595;
            o[1, 2] = 710;
            o[0, 3] = 720;
            o[1, 3] = 860;
            o[0, 4] = 845;
            o[1, 4] = 1010;
            Assert.AreEqual(o, m * n);
            n = new Matrix(5, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
            n = new Matrix(3, 2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = n * m; });
        }

        [Test]
        [TestOf(typeof(CovarianceMatrix))]
        public void CovarianceMatrixTestInverse() {
            CovarianceMatrix m = new CovarianceMatrix(2),
                   o = new CovarianceMatrix(2);
            //Inverse of an empty Matrix:
            Assert.AreEqual(null, !((CovarianceMatrix) null));
            //Inverse:
            m[0, 0][0, 0] = 3.0f;
            m[0, 0][1, 0] = 5.0f;
            m[0, 0][2, 0] = 2.0f;
            m[1, 0][0, 0] = 1.0f;
            m[1, 0][1, 0] = 11.0f;
            m[0, 0][0, 1] = 10.0f;
            m[0, 0][1, 1] = 7.0f;
            m[0, 0][2, 1] = 6.0f;
            m[1, 0][0, 1] = 7.0f;
            m[1, 0][1, 1] = 12.0f;
            m[0, 0][0, 2] = 2.0f;
            m[0, 0][1, 2] = 2.0f;
            m[0, 0][2, 2] = 2.0f;
            m[1, 0][0, 2] = 2.0f;
            m[1, 0][1, 2] = 2.0f;
            m[0, 1][0, 0] = 3.0f;
            m[0, 1][1, 0] = 4.0f;
            m[0, 1][2, 0] = 5.0f;
            m[1, 1][0, 0] = 6.0f;
            m[1, 1][1, 0] = 7.0f;
            m[0, 1][0, 1] = 9.0f;
            m[0, 1][1, 1] = 4.0f;
            m[0, 1][2, 1] = 2.0f;
            m[1, 1][0, 1] = 7.0f;
            m[1, 1][1, 1] = 8.0f;

            o[0, 0][0, 0] = -9f / 77f;
            o[0, 0][1, 0] = 85f / 308f;
            o[0, 0][2, 0] = -155f / 616f;
            o[1, 0][0, 0] = -51f / 308f;
            o[1, 0][1, 0] = -1f / 22f;
            o[0, 0][0, 1] = 23f / 77f;
            o[0, 0][1, 1] = -183f / 308f;
            o[0, 0][2, 1] = 1149f / 616f;
            o[1, 0][0, 1] = -75f / 308f;
            o[1, 0][1, 1] = 5f / 22f;
            o[0, 0][0, 2] = -17f / 77f;
            o[0, 0][1, 2] = 38f / 77f;
            o[0, 0][2, 2] = -101f / 154f;
            o[1, 0][0, 2] = 8f / 77f;
            o[1, 0][1, 2] = -4f / 11f;
            o[0, 1][0, 0] = 1f / 77f;
            o[0, 1][1, 0] = -95f / 308f;
            o[0, 1][2, 0] = 137f / 616f;
            o[1, 1][0, 0] = 57f / 308f;
            o[1, 1][1, 0] = 5f / 22f;
            o[0, 1][0, 1] = 2f / 77f;
            o[0, 1][1, 1] = 41f / 308f;
            o[0, 1][2, 1] = -419f / 616f;
            o[1, 1][0, 1] = 37f / 308f;
            o[1, 1][1, 1] = -1f / 22f;

            var p = !m;
            Assert.AreEqual(o, p);
            p = !o;
            Assert.AreEqual(m, p);
            //Inverse of identity:
            m = new CovarianceMatrix(5, 1.0f);
            p = !m;
            Assert.AreEqual(m, p);
        }
    #endregion
    #region SparseColumn
        [Test]
        [TestOf(typeof(SparseColumn))]
        public void SparseColumnTestValues() {
            SparseColumn c = new SparseColumn();
            Assert.IsTrue(c.val.Keys.Count == 0);
            c[10] = new Matrix(5);
            Assert.IsTrue(c.val.Keys.Count == 1);
            Assert.AreEqual(new Matrix(5), c[10]);
            Assert.AreEqual(null, c[9]);
            Assert.AreEqual(null, c[0]);
            c.Remove(0);
            Assert.IsTrue(c.val.Keys.Count == 1);
            Assert.AreEqual(null, c[0]);
            Assert.AreEqual(new Matrix(5), c[10]);
            c.Remove(10);
            Assert.IsTrue(c.val.Keys.Count == 0);
            Assert.AreEqual(null, c[10]);
            c[9] = new Matrix(5);
            Assert.IsTrue(c.val.Keys.Count == 1);
            Assert.AreEqual(new Matrix(5), c[9]);
            c.Clear();
            Assert.IsTrue(c.val.Keys.Count == 0);
            Assert.AreEqual(null, c[9]);
        }

        [Test]
        [TestOf(typeof(SparseColumn))]
        public void SparseColumnTestAddition() {
            SparseColumn c = new SparseColumn(),
                         d = new SparseColumn();
            Assert.AreEqual(c, c + ((SparseColumn) null));
            Assert.AreEqual(c, ((SparseColumn) null) + c);
            Assert.AreEqual(null, ((SparseColumn) null) + ((SparseColumn) null));
            Assert.AreEqual(c, c + ((SparseColumn) null));
            Assert.AreEqual(c, c + d);
            c[10] = new Matrix(5);
            c[14] = new Matrix(4);
            Assert.AreEqual(c, c + d);
            d[0] = new Matrix(3, 3);
            d[2] = new Matrix(3, 3);
            d[5] = new Matrix(3, 3);
            Assert.AreNotEqual(c, c + d);
            c[0] = new Matrix(3);
            c[2] = new Matrix(3);
            c[5] = new Matrix(3);
            d[10] = new Matrix(5, 5);
            d[14] = new Matrix(4, 4);
            Assert.AreEqual(c, c + d);
            c.Clear();
            d.Clear();
            c.Addition(null);
            Assert.AreEqual(d, c);
            c.Addition(d);
            Assert.AreEqual(d, c);
            c[10] = new Matrix(5);
            c[14] = new Matrix(4);
            d[0] = new Matrix(3, 3);
            d[2] = new Matrix(3, 3);
            d[5] = new Matrix(3, 3);
            SparseColumn e = new SparseColumn();
            e[10] = new Matrix(5);
            e[14] = new Matrix(4);
            e[0] = new Matrix(3, 3);
            e[2] = new Matrix(3, 3);
            e[5] = new Matrix(3, 3);
            c.Addition(d);
            Assert.AreEqual(e, c);
        }
    #endregion
    #region SparseMatrix
        [Test]
        [TestOf(typeof(SparseMatrix))]
        public void SparseMatrixTestValues() {
            SparseMatrix c = new SparseMatrix();
            Assert.IsTrue(c.RowCount() == 0);
            Assert.IsTrue(c.ColumnCount() == 0);
            c.Enlarge(10);
            Assert.IsTrue(c.RowCount() == 0);
            Assert.IsTrue(c.ColumnCount() == 10);
            c.AddColumn(new SparseColumn());
            Assert.IsTrue(c.RowCount() == 0);
            Assert.IsTrue(c.ColumnCount() == 11);
            c.SetRowCount(1234);
            Assert.IsTrue(c.RowCount() == 1234);
            Assert.AreEqual(null, c[4, 8]);
            c[4, 8] = new Matrix(23, 12);
            Matrix m = new Matrix(23, 12);
            Assert.AreEqual(m, c[4, 8]);
            SparseColumn d = c.GetColumn(4);
            Assert.AreEqual(m, d[8]);
        }

        [Test]
        [TestOf(typeof(SparseMatrix))]
        public void SparseMatrixTestIFeatureMultiplication() {
            SparseMatrix m = new SparseMatrix();
            List<IFeature> n = new List<IFeature>();
            SparseColumn o = new SparseColumn();
            Assert.AreEqual(o, m * n);
            Assert.AreEqual(null, ((SparseMatrix) null) * n);
            Assert.AreEqual(null, m * ((List<IFeature>) null));
            Assert.AreEqual(null, ((SparseMatrix) null) * ((List<IFeature>) null));
            m.Enlarge(RANDOM_ITERATIONS);
            m[0, 0] = new Matrix(3);
            int i;
            n.Add(new RobotPose(new Vector3(1.0f,2.0f,3.0f),-10f));
            o[0] = new Matrix(1,3);
            o[0][0, 0] = 1.0f;
            o[0][0, 1] = 2.0f;
            o[0][0, 2] = 3.0f;
            for(i = 1; i < RANDOM_ITERATIONS; i++) {
                m[i, i] = new Matrix(2);
                n.Add(new Feature(new Vector2(2f+2f*i,3f+2f*i),null, i));
                o[i] = new Matrix(1,2);
                o[i][0, 0] = 2f+2f*i;
                o[i][0, 1] = 3f+2f*i;
            }
            Assert.AreEqual(o, m * n);
            m = new SparseMatrix();
            m.Enlarge(2);
            m[0, 0] = new Matrix(3);
            m[0, 1] = new Matrix(3, 2);
            m[1, 0] = new Matrix(2, 3);
            m[1, 1] = new Matrix(2);
            i = 0;
            int j = 0,
                k = 0,
                l = 0;
            while(true) {
                m[i, j][k, l] = 6 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            n = new List<IFeature>();
            n.Add(new RobotPose(new Vector3(1.0f,2.0f,3.0f),-10f));
            n.Add(new Feature(new Vector2(4f,5f),null, 1));
            o = new SparseColumn();
            o[0] = new Matrix(1,3);
            o[0][0, 0] = 130.0f;
            o[0][0, 1] = 205.0f;
            o[0][0, 2] = 280.0f;
            o[1] = new Matrix(1,2);
            o[1][0, 0] = 355;
            o[1][0, 1] = 430;
            Assert.AreEqual(o, m * n);
            n = new List<IFeature>();
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
            n = new List<IFeature>();
            n.Add(new Feature(new Vector2(5f,6f),null, 0));
            n.Add(new RobotPose(new Vector3(1.0f,2.0f,3.0f),-10f));
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
        }

        [Test]
        [TestOf(typeof(SparseMatrix))]
        public void SparseMatrixTestSparseColumnMultiplication() {
            SparseMatrix m = new SparseMatrix();
            SparseColumn n = new SparseColumn();
            SparseColumn o = new SparseColumn();
            Assert.AreEqual(o, m * n);
            Assert.AreEqual(null, ((SparseMatrix) null) * n);
            Assert.AreEqual(null, m * ((List<IFeature>) null));
            Assert.AreEqual(null, ((SparseMatrix) null) * ((List<IFeature>) null));
            m.Enlarge(RANDOM_ITERATIONS);
            m[0, 0] = new Matrix(3);
            int i;
            n[0] = new Matrix(1,3);
            n[0][0, 0] = 1.0f;
            n[0][0, 1] = 2.0f;
            n[0][0, 2] = 3.0f;
            o[0] = new Matrix(1,3);
            o[0][0, 0] = 1.0f;
            o[0][0, 1] = 2.0f;
            o[0][0, 2] = 3.0f;
            for(i = 1; i < RANDOM_ITERATIONS; i++) {
                m[i, i] = new Matrix(2);
                n[i] = new Matrix(1,2);
                n[i][0, 0] = 2f+2f*i;
                n[i][0, 1] = 3f+2f*i;
                o[i] = new Matrix(1,2);
                o[i][0, 0] = 2f+2f*i;
                o[i][0, 1] = 3f+2f*i;
            }
            Assert.AreEqual(o, m * n);
            m = new SparseMatrix();
            m.Enlarge(2);
            m[0, 0] = new Matrix(3);
            m[0, 1] = new Matrix(3, 2);
            m[1, 0] = new Matrix(2, 3);
            m[1, 1] = new Matrix(2);
            i = 0;
            int j = 0,
                k = 0,
                l = 0;
            while(true) {
                m[i, j][k, l] = 6 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            n = new SparseColumn();
            n[0] = new Matrix(1,3);
            n[0][0, 0] = 1.0f;
            n[0][0, 1] = 2.0f;
            n[0][0, 2] = 3.0f;
            n[1] = new Matrix(1,2);
            n[1][0, 0] = 4f;
            n[1][0, 1] = 5f;
            o = new SparseColumn();
            o[0] = new Matrix(1,3);
            o[0][0, 0] = 130.0f;
            o[0][0, 1] = 205.0f;
            o[0][0, 2] = 280.0f;
            o[1] = new Matrix(1,2);
            o[1][0, 0] = 355;
            o[1][0, 1] = 430;
            Assert.AreEqual(o, m * n);
            n = new SparseColumn();
            n[0] = new Matrix(1,2);
            n[0][0, 0] = 4f;
            n[0][0, 1] = 5f;
            n[1] = new Matrix(1,3);
            n[1][0, 0] = 1.0f;
            n[1][0, 1] = 1.0f;
            n[1][0, 2] = 1.0f;
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
        }

        [Test]
        [TestOf(typeof(SparseMatrix))]
        public void SparseMatrixTestSparseMatrixMultiplication() {
            SparseMatrix m = new SparseMatrix(),
                         n = new SparseMatrix(),
                         o = new SparseMatrix();
            Assert.AreEqual(null, m * ((SparseMatrix) null));
            Assert.AreEqual(null, ((SparseMatrix) null) * n);
            Assert.AreEqual(null, ((SparseMatrix) null) * ((SparseMatrix) null));
            Assert.AreEqual(o, m * n);
            m.Enlarge(RANDOM_ITERATIONS);
            n.Enlarge(RANDOM_ITERATIONS);
            o.Enlarge(RANDOM_ITERATIONS);
            m.SetRowCount(RANDOM_ITERATIONS);
            n.SetRowCount(RANDOM_ITERATIONS);
            System.Random r = new System.Random();
            int i;
            for(i = 0; i < RANDOM_ITERATIONS; i++) {
                int size = r.Next(2,4);
                m[i, i] = new Matrix(size);
                n[i, i] = new Matrix(size);
                o[i, i] = new Matrix(size);
            }
            var p = m * n;
            Assert.IsTrue(p.RowCount() == RANDOM_ITERATIONS);
            Assert.IsTrue(p.ColumnCount() == RANDOM_ITERATIONS);
            Assert.AreEqual(o, m * n);
            m = new SparseMatrix();
            m.Enlarge(2);
            m.SetRowCount(2);
            m[0, 0] = new Matrix(3);
            m[0, 1] = new Matrix(3, 2);
            m[1, 0] = new Matrix(2, 3);
            m[1, 1] = new Matrix(2);
            n = new SparseMatrix();
            n.Enlarge(2);
            n.SetRowCount(2);
            n[0, 0] = new Matrix(3);
            n[0, 1] = new Matrix(3, 2);
            n[1, 0] = new Matrix(2, 3);
            n[1, 1] = new Matrix(2);
            o = new SparseMatrix();
            o.Enlarge(2);
            o[0, 0] = new Matrix(3);
            o[0, 1] = new Matrix(3, 2);
            o[1, 0] = new Matrix(2, 3);
            o[1, 1] = new Matrix(2);
            i = 0;
            int j = 0,
                k = 0,
                l = 0;
            while(true) {
                m[i, j][k, l] = 6 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                n[i, j][k, l] = 31 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            o[0, 0][0, 0] = 1690;
            o[0, 0][1, 0] = 1730;
            o[0, 0][2, 0] = 1770;
            o[1, 0][0, 0] = 1810;
            o[1, 0][1, 0] = 1850;
            o[0, 0][0, 1] = 2715;
            o[0, 0][1, 1] = 2780;
            o[0, 0][2, 1] = 2845;
            o[1, 0][0, 1] = 2910;
            o[1, 0][1, 1] = 2975;
            o[0, 0][0, 2] = 3740;
            o[0, 0][1, 2] = 3830;
            o[0, 0][2, 2] = 3920;
            o[1, 0][0, 2] = 4010;
            o[1, 0][1, 2] = 4100;
            o[0, 1][0, 0] = 4765;
            o[0, 1][1, 0] = 4880;
            o[0, 1][2, 0] = 4995;
            o[1, 1][0, 0] = 5110;
            o[1, 1][1, 0] = 5225;
            o[0, 1][0, 1] = 5790;
            o[0, 1][1, 1] = 5930;
            o[0, 1][2, 1] = 6070;
            o[1, 1][0, 1] = 6210;
            o[1, 1][1, 1] = 6350;
            Assert.AreEqual(o, m * n);
            n = new SparseMatrix();
            n.Enlarge(2);
            n.SetRowCount(12345);
            Assert.Throws<MatrixSizeException>(() => { var tmp = m * n; });
            n.Enlarge(12);
            n.SetRowCount(2);
            Assert.Throws<MatrixSizeException>(() => { var tmp = n * m; });
        }

        [Test]
        [TestOf(typeof(SparseMatrix))]
        public void SparseMatrixTestTranslate() {
            Assert.AreEqual(null, ~((SparseMatrix) null));
            SparseMatrix c = new SparseMatrix();
            c.Enlarge(RANDOM_ITERATIONS);
            c.SetRowCount(RANDOM_ITERATIONS);
            Assert.AreEqual(c, ~~c);
             System.Random r = new System.Random();
            for(int i = 0; i < RANDOM_ITERATIONS; i++) {
                c[i, i] = new Matrix(r.Next(2,4));
            }
            Assert.AreEqual(c, ~~c);
        }
    #endregion
    #region SparseTranslatedMatrix
        [Test]
        [TestOf(typeof(SparseTranslatedMatrix))]
        public void SparseTranslatedMatrixTestValues() {
            SparseMatrix c = new SparseMatrix();
            c.Enlarge(10);
            c.SetRowCount(1234);
            c[4, 8] = new Matrix(23, 12);
            SparseTranslatedMatrix d = new SparseTranslatedMatrix(c);
            Assert.IsTrue(d.RowCount() == 10);
            Assert.IsTrue(d.ColumnCount() == 1234);

            Matrix m = new Matrix(23, 12);
            SparseColumn e = d.GetRow(4);
            Assert.AreEqual(m, e[8]);
        }

        [Test]
        [TestOf(typeof(SparseTranslatedMatrix))]
        public void SparseTranslatedMatrixTestTranslate() {
            Assert.AreEqual(null, ~((SparseTranslatedMatrix) null));
            SparseMatrix c = new SparseMatrix();
            c.Enlarge(10);
            c.SetRowCount(1234);
            c[4, 8] = new Matrix(23, 12);
            SparseTranslatedMatrix d = new SparseTranslatedMatrix(c);
            Assert.AreEqual(c, ~d);
        }
        #endregion
    #region SparseCovarianceMatrix
        [Test]
        [TestOf(typeof(SparseCovarianceMatrix))]
        public void SparseCovarianceMatrixTestValues() {
            SparseCovarianceMatrix m = new SparseCovarianceMatrix();
            Assert.IsTrue(m.val.Count == 0);
            Assert.IsTrue(m.ColumnCount() == 0);
            m.Enlarge(10);
            Assert.IsTrue(m.val.Count == 10);
            Assert.IsTrue(m.ColumnCount() == 10);
            for(int i = 0; i < 10; i++) Assert.AreEqual(new SparseColumn(), m.GetColumn(i));
            m.Add(new SparseColumn());
            Assert.IsTrue(m.val.Count == 11);
            Assert.IsTrue(m.ColumnCount() == 11);
            Assert.AreEqual(new SparseColumn(), m.GetColumn(10));
            m.Clear();
            Assert.IsTrue(m.val.Count == 11);
            Assert.IsTrue(m.ColumnCount() == 11);
        }

        [Test]
        [TestOf(typeof(SparseCovarianceMatrix))]
        public void SparseCovarianceMatrixTestTrim() {
            SparseCovarianceMatrix m = new SparseCovarianceMatrix(),
                                   n = new SparseCovarianceMatrix();
            LinkedList<int> rows = new LinkedList<int>();
            m.Enlarge(RANDOM_ITERATIONS);
            n.Enlarge(RANDOM_ITERATIONS);
            System.Random r = new System.Random();
            int j = 0;
            for(int i = 0; i < RANDOM_ITERATIONS; i++) {
                m[i, i] = new Matrix(1, 1);
                m[i, i][0, 0] = i;
                if(r.Next(2) == 1) {
                    rows.AddLast(i);
                    n[i, j] = new Matrix(1, 1);
                    n[i, j][0, 0] = i;
                    j++;
                }
            }
            m.Trim(rows, RANDOM_ITERATIONS);
            Assert.AreEqual(n, m);
            Assert.Throws<System.ArgumentException>(() => { m.Trim(new LinkedList<int>(), RANDOM_ITERATIONS); });
        }

        [Test]
        [TestOf(typeof(SparseCovarianceMatrix))]
        public void SparseCovarianceMatrixTestAddition() {
            SparseCovarianceMatrix m = new SparseCovarianceMatrix(),
                                   n = new SparseCovarianceMatrix();
            SparseMatrix o = new SparseMatrix();
            m.Enlarge(SPARSE_ITERATIONS);
            n.Enlarge(SPARSE_ITERATIONS);
            o.Enlarge(SPARSE_ITERATIONS);
            System.Random r = new System.Random();
            for(int i = 0; i < SPARSE_ITERATIONS; i++) {
                for(int j = 0; j < SPARSE_ITERATIONS; j++) {
                    var val1 = r.Next(RANDOM_ITERATIONS);
                    m[i, j] = new Matrix(1, 1);
                    m[i, j][0, 0] = (float) val1;
                    n[i, j] = new Matrix(1, 1);
                    n[i, j][0, 0] = (float) val1;
                }
            }
            m.Addition(null);
            Assert.AreEqual(m, n);
            m.Addition(o);
            Assert.AreEqual(m, n);
            m.Clear();
            n.Clear();
            for(int i = 0; i < SPARSE_ITERATIONS; i++) {
                for(int j = 0; j < SPARSE_ITERATIONS; j++) {
                    var val1 = r.Next(RANDOM_ITERATIONS);
                    var val2 = r.Next(RANDOM_ITERATIONS);
                    m[i, j] = new Matrix(1, 1);
                    m[i, j][0, 0] = (float) val1;
                    n[i, j] = new Matrix(1, 1);
                    n[i, j][0, 0] = (float) (val1 + val2);
                    o[i, j] = new Matrix(1, 1);
                    o[i, j][0, 0] = (float) val2;
                }
            }
            m.Addition(o);
            Assert.AreEqual(n, m);
            o = new SparseMatrix();
            Assert.Throws<MatrixSizeException>(() => { m.Addition(o); });
        }

        [Test]
        [TestOf(typeof(SparseCovarianceMatrix))]
        public void SparseCovarianceMatrixTestMultiplication() {
            SparseCovarianceMatrix m = new SparseCovarianceMatrix();
            SparseMatrix n = new SparseMatrix(),
                         o = new SparseMatrix();
            Assert.AreEqual(null, ~n * (SparseCovarianceMatrix) null);
            Assert.AreEqual(null, ((SparseTranslatedMatrix) null) * m);
            Assert.AreEqual(null, ((SparseTranslatedMatrix) null) * ((SparseCovarianceMatrix) null));
            Assert.AreEqual(o, ~n * m);
            m.Enlarge(RANDOM_ITERATIONS);
            n.Enlarge(RANDOM_ITERATIONS);
            o.Enlarge(RANDOM_ITERATIONS);
            n.SetRowCount(RANDOM_ITERATIONS);
            System.Random r = new System.Random();
            int i;
            for(i = 0; i < RANDOM_ITERATIONS; i++) {
                int size = r.Next(2,4);
                m[i, i] = new Matrix(size);
                n[i, i] = new Matrix(size);
                o[i, i] = new Matrix(size);
            }
            m = new SparseCovarianceMatrix();
            m.Enlarge(2);
            m[0, 0] = new Matrix(3);
            m[0, 1] = new Matrix(3, 2);
            m[1, 0] = new Matrix(2, 3);
            m[1, 1] = new Matrix(2);
            n = new SparseMatrix();
            n.Enlarge(2);
            n.SetRowCount(2);
            n[0, 0] = new Matrix(3);
            n[0, 1] = new Matrix(3, 2);
            n[1, 0] = new Matrix(2, 3);
            n[1, 1] = new Matrix(2);
            o = new SparseMatrix();
            o.Enlarge(2);
            o[0, 0] = new Matrix(3);
            o[0, 1] = new Matrix(3, 2);
            o[1, 0] = new Matrix(2, 3);
            o[1, 1] = new Matrix(2);
            i = 0;
            int j = 0,
                k = 0,
                l = 0;
            while(true) {
                m[i, j][k, l] = 6 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                n[i, j][k, l] = 31 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            o[0, 0][0, 0] = 3530;
            o[0, 0][1, 0] = 3735;
            o[0, 0][2, 0] = 3940;
            o[1, 0][0, 0] = 4145;
            o[1, 0][1, 0] = 4350;
            o[0, 0][0, 1] = 3610;
            o[0, 0][1, 1] = 3820;
            o[0, 0][2, 1] = 4030;
            o[1, 0][0, 1] = 4240;
            o[1, 0][1, 1] = 4450;
            o[0, 0][0, 2] = 3690;
            o[0, 0][1, 2] = 3905;
            o[0, 0][2, 2] = 4120;
            o[1, 0][0, 2] = 4335;
            o[1, 0][1, 2] = 4550;
            o[0, 1][0, 0] = 3770;
            o[0, 1][1, 0] = 3990;
            o[0, 1][2, 0] = 4210;
            o[1, 1][0, 0] = 4430;
            o[1, 1][1, 0] = 4650;
            o[0, 1][0, 1] = 3850;
            o[0, 1][1, 1] = 4075;
            o[0, 1][2, 1] = 4300;
            o[1, 1][0, 1] = 4525;
            o[1, 1][1, 1] = 4750;
            Assert.AreEqual(o, ~n * m);
            n = new SparseMatrix();
            n.Enlarge(2);
            n.SetRowCount(12345);
            Assert.Throws<MatrixSizeException>(() => { var tmp = ~n * m; });
        }
    #endregion
    #region SparseTriangularMatrix
        [Test]
        [TestOf(typeof(SparseTriangularMatrix))]
        public void SparseTriangularMatrixTestValues() {
            SparseTriangularMatrix m = new SparseTriangularMatrix();
            Assert.IsTrue(m.ColumnCount() == 0);
            m.Enlarge(10);
            Assert.IsTrue(m.ColumnCount() == 10);
            int i = 0;
            for(; i < 10; i++) m[i, i] = new Matrix(3);
            for(i = 6; i < 10; i++) m[5, i] = new Matrix(2);
            Matrix o = new Matrix(3);
            Matrix p = new Matrix(2);
            for (i = 0; i < 10; i++) {
                for(int j = i; j < 10; j++) {
                    if(i == j) Assert.AreEqual(o, m[i, i]);
                    else if(i == 5) Assert.AreEqual(p, m[5, j]);
                    else Assert.AreEqual(null, m[i, j]);
                }
            }
            IEnumerator<SparseColumn> enumerator = m.GetColumnEnumerator();
            i = 0;
            while(enumerator.MoveNext()) {
                for(int j = i; j < 10; j++) {
                    if(i == j) Assert.AreEqual(o, m[i, i]);
                    else if(i == 5) Assert.AreEqual(p, m[5, j]);
                    else Assert.AreEqual(null, m[i, j]);
                }
                i++;
            }
            enumerator.Dispose();
        }

        [Test]
        [TestOf(typeof(SparseTriangularMatrix))]
        public void SparseTriangularMatrixTestSolveLowerLeftSparse() {
            SparseTriangularMatrix m = new SparseTriangularMatrix();
            SparseColumn c = new SparseColumn(),
                         d = new SparseColumn();
            m.Enlarge(2);
            m[0, 0] = new Matrix(3);
            m[0, 1] = new Matrix(3, 2);
            m[1, 0] = new Matrix(2, 3);
            m[1, 1] = new Matrix(2);
            Assert.AreEqual(d, m.solveLowerLeftSparse(c));
            int i = 0,
                j = 0,
                k = 0,
                l = 0;
            while(true) {
                if((i == j && k <= l) || (i < j)) m[i, j][k, l] = 6 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            c[0] = new Matrix(1,3);
            c[1] = new Matrix(1,2);
            d[0] = new Matrix(1,3);
            d[1] = new Matrix(1,2);
            c[0][0, 0] = 1f;
            c[0][0, 1] = 2f;
            c[0][0, 2] = 3f;
            c[1][0, 0] = 4f;
            c[1][0, 1] = 5f;
            d[0][0, 0] = 1f/ 6f;
            d[0][0, 1] = 1f / 72f;
            d[0][0, 2] = 7f / 1296f;
            d[1][0, 0] = 91f / 31104f;
            d[1][0, 1] = 1729f / 933120f;
            Assert.AreEqual(d, m.solveLowerLeftSparse(c));
        }

        [Test]
        [TestOf(typeof(SparseTriangularMatrix))]
        public void SparseTriangularMatrixTestSolveUpperRightSparse() {
            SparseTriangularMatrix m = new SparseTriangularMatrix();
            SparseColumn c = new SparseColumn(),
                         d = new SparseColumn();
            m.Enlarge(2);
            m[0, 0] = new Matrix(3);
            m[0, 1] = new Matrix(3, 2);
            m[1, 0] = new Matrix(2, 3);
            m[1, 1] = new Matrix(2);
            Assert.AreEqual(d, m.solveUpperRightSparse(c));
            int i = 0,
                j = 0,
                k = 0,
                l = 0;
            while(true) {
                if((i == j && k <= l) || (i < j)) m[i, j][k, l] = 6 + (i > 0 ? i * 2 + 1 : 0) + (j > 0 ? j * 10 + 5 : 0) + k + l * 5;
                k++;
                if(i > 0) {
                    if(k > 1) {
                        k = 0;
                        i++;
                        if(i > 1) {
                            i = 0;
                            l++;
                            if(j > 0) {
                                if(l > 1) {
                                    l = 0;
                                    j++;
                                    if(j > 1) break;
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
            c[0] = new Matrix(1,3);
            c[1] = new Matrix(1,2);
            d[0] = new Matrix(1,3);
            d[1] = new Matrix(1,2);
            c[0][0, 0] = 1f;
            c[0][0, 1] = 2f;
            c[0][0, 2] = 3f;
            c[1][0, 0] = 4f;
            c[1][0, 1] = 5f;
            d[0][0, 0] = -30875f / 186624f;
            d[0][0, 1] = -2375f / 31104f;
            d[0][0, 2] = -125f / 2592f;
            d[1][0, 0] = -5f /144f;
            d[1][0, 1] = 1f/ 6f;
            Assert.AreEqual(d, m.solveUpperRightSparse(c));
        }
    #endregion
    #region DefaultedSparseFloatMatrix
        [Test]
        [TestOf(typeof(DefaultedSparseFloatMatrix))]
        public void DefaultedSparseFloatMatrixTestValues() {
            DefaultedSparseFloatMatrix m = new DefaultedSparseFloatMatrix(float.NaN);
            Assert.IsTrue(m.ColumnCount() == 0);
            m.Enlarge(10);
            Assert.IsTrue(m.ColumnCount() == 10);
            int i = 0;
            for(; i < 10; i++) m[i, i] = i;
            for(i = 6; i < 10; i++) m[5, i] = 5;
            for (i = 0; i < 10; i++) {
                for(int j = i; j < 10; j++) {
                    if(i == j) Assert.AreEqual(i, m[i, i]);
                    else if(i == 5) Assert.AreEqual(5, m[5, j]);
                    else Assert.IsTrue(float.IsNaN(m[i, j]));
                }
            }
            IEnumerator<KeyValuePair<int, float>> enumerator = m.GetColumn(5);
            i = 0;
            while(enumerator.MoveNext()) {
                Assert.AreEqual(5, enumerator.Current.Value);
                i++;
            }
            enumerator.Dispose();
        }
    #endregion
    #region SparseFloatColumn
        [Test]
        [TestOf(typeof(SparseFloatColumn))]
        public void SparseFloatColumnTestValues() {
            SparseFloatColumn m = new SparseFloatColumn(float.NaN);
            int i = 0;
            for(; i < 10; i++) m[i] = i;
            for (i = 0; i < 10; i++) Assert.AreEqual(i, m[i]);
            for (i = 10; i < 20; i++) Assert.IsTrue(float.IsNaN(m[i]));
        }
    }
    #endregion
}

#pragma warning restore 0219
