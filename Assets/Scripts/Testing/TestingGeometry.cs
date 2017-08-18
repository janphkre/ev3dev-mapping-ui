using System;
using ev3devMapping.Society;
using NUnit.Framework;
using UnityEngine;

namespace ev3devMapping.Testing {

    public class TestingGeometry : ITesting {

        [Test]
        [TestOf(typeof(Geometry))]
        public void GeometryTestEuclidean() {
            //Vector2, Vector2
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(Vector2.zero, Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector2(0.0f, 1.0f), Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector2(1.0f, 0.0f), Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector2.zero, new Vector2(1.0f, 0.0f)), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector2.zero, new Vector2(0.0f, 1.0f)), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(new Vector2(1.0f, 1.0f), Vector2.zero), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(Vector2.zero, new Vector2(1.0f, 1.0f)), DELTA);

            Assert.AreEqual(Math.Sqrt(149.0d), Geometry.EuclideanDistance(new Vector2(10.0f, 10.0f), new Vector2(20.0f, 17.0f)), DELTA);
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(new Vector2(23.5f, 23.5f), new Vector2(23.5f, 23.5f)), DELTA);
            Assert.AreEqual(153.08, Geometry.EuclideanDistance(new Vector2(48.5f, -11.9f), new Vector2(-66.8f, 88.8f)), 0.01d);

            //Vector3, Vector3
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(Vector3.zero, Vector3.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector3(0.0f, 1.0f), Vector3.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector3(1.0f, 0.0f), Vector3.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector3.zero, new Vector3(1.0f, 0.0f)), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector3.zero, new Vector3(0.0f, 1.0f)), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(new Vector3(1.0f, 1.0f), Vector3.zero), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(Vector3.zero, new Vector3(1.0f, 1.0f)), DELTA);

            Assert.AreEqual(Math.Sqrt(149.0d), Geometry.EuclideanDistance(new Vector3(10.0f, 10.0f), new Vector3(20.0f, 17.0f)), DELTA);
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(new Vector3(23.5f, 23.5f), new Vector3(23.5f, 23.5f)), DELTA);
            Assert.AreEqual(153.08, Geometry.EuclideanDistance(new Vector3(48.5f, -11.9f), new Vector3(-66.8f, 88.8f)), 0.01d);

            //Vector3, Vector2
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(Vector3.zero, Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector3(0.0f, float.NaN, 1.0f), Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector3(1.0f, float.NaN, 0.0f), Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector3.zero, new Vector2(1.0f, 0.0f)), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector3.zero, new Vector2(0.0f, 1.0f)), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(new Vector3(1.0f, 0.0f, 1.0f), Vector2.zero), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(Vector3.zero, new Vector2(1.0f, 1.0f)), DELTA);

            Assert.AreEqual(Math.Sqrt(149.0d), Geometry.EuclideanDistance(new Vector3(10.0f, float.NaN, 10.0f), new Vector2(20.0f, 17.0f)), DELTA);
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(new Vector3(23.5f, float.NaN, 23.5f), new Vector2(23.5f, 23.5f)), DELTA);
            Assert.AreEqual(153.08, Geometry.EuclideanDistance(new Vector3(48.5f, float.NaN, -11.9f), new Vector2(-66.8f, 88.8f)), 0.01d);

            //GraphNode, GraphNode
            GraphNode a = new GraphNode(Vector2.zero, float.NaN),
                      b = new GraphNode(Vector2.zero, float.NaN);
            RobotPose poseA = new RobotPose(new Vector3(1.0f, 1.0f, float.NaN), float.NaN),
                      poseB = new RobotPose(new Vector3(-1.0f, -1.0f, float.NaN), float.NaN);
            //both poses are null:
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(0.0f, 1.0f);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(1.0f, 0.0f);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(1.0f, 1.0f);
            b.centerOffset = Vector2.zero;
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(b, a), DELTA);

            a.centerOffset = new Vector2(10.0f, 10.0f);
            b.centerOffset = new Vector2(20.0f, 17.0f);
            Assert.AreEqual(Math.Sqrt(149.0d), Geometry.EuclideanDistance(a, b), DELTA);
            a.centerOffset = new Vector2(23.5f, 23.5f);
            b.centerOffset = new Vector2(23.5f, 23.5f);
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(a, b), DELTA);
            a.centerOffset = new Vector2(48.5f, -11.9f);
            b.centerOffset = new Vector2(-66.8f, 88.8f);
            Assert.AreEqual(153.08d, Geometry.EuclideanDistance(a, b), DELTA_2);

            //Second pose is still null:
            a.pose = poseA;
            a.centerOffset = Vector2.zero;
            b.centerOffset = Vector2.zero;
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(0.0f, 1.0f);
            Assert.AreEqual(Math.Sqrt(1.0d+4.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(1.0d+4.0d), Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(1.0f, 0.0f);
            Assert.AreEqual(Math.Sqrt(1.0d+4.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(1.0d+4.0d), Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(1.0f, 1.0f);
            b.centerOffset = Vector2.zero;
            Assert.AreEqual(Math.Sqrt(8.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(8.0d), Geometry.EuclideanDistance(b, a), DELTA);

            a.centerOffset = new Vector2(10.0f, 10.0f);
            b.centerOffset = new Vector2(20.0f, 17.0f);
            Assert.AreEqual(10.82d, Geometry.EuclideanDistance(a, b), DELTA_2);
            a.centerOffset = new Vector2(23.5f, 23.5f);
            b.centerOffset = new Vector2(23.5f, 23.5f);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(a, b), DELTA);
            a.centerOffset = new Vector2(48.5f, -11.9f);
            b.centerOffset = new Vector2(-66.8f, 88.8f);
            Assert.AreEqual(153.19d, Geometry.EuclideanDistance(a, b), DELTA_2);

            //No pose is null:
            b.pose = poseB;
            a.centerOffset = Vector2.zero;
            b.centerOffset = Vector2.zero;
            Assert.AreEqual(Math.Sqrt(8.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(8.0d), Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(0.0f, 1.0f);
            Assert.AreEqual(Math.Sqrt(4.0d+9.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(4.0d+9.0d), Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(1.0f, 0.0f);
            Assert.AreEqual(Math.Sqrt(4.0d+9.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(4.0d+9.0d), Geometry.EuclideanDistance(b, a), DELTA);
            a.centerOffset = new Vector2(1.0f, 1.0f);
            b.centerOffset = Vector2.zero;
            Assert.AreEqual(Math.Sqrt(18.0d), Geometry.EuclideanDistance(a, b), DELTA);
            Assert.AreEqual(Math.Sqrt(18.0d), Geometry.EuclideanDistance(b, a), DELTA);

            a.centerOffset = new Vector2(10.0f, 10.0f);
            b.centerOffset = new Vector2(20.0f, 17.0f);
            Assert.AreEqual(9.43d, Geometry.EuclideanDistance(a, b), DELTA_2);
            a.centerOffset = new Vector2(23.5f, 23.5f);
            b.centerOffset = new Vector2(23.5f, 23.5f);
            Assert.AreEqual(Math.Sqrt(8.0d), Geometry.EuclideanDistance(a, b), DELTA);
            a.centerOffset = new Vector2(48.5f, -11.9f);
            b.centerOffset = new Vector2(-66.8f, 88.8f);
            Assert.AreEqual(153.3d, Geometry.EuclideanDistance(a, b), DELTA_2);

        }

        [Test]
        [TestOf(typeof(Geometry))]
        public void GeometryTestMahalanobis() {
            //For an identity matrix the mahalanobis distance is the euclidean distance:
            Matrix m = new Matrix(2);
            Assert.AreEqual(0.0d, Geometry.RealMahalanobisDistance(Vector2.zero, Vector2.zero, m), DELTA);

            System.Random r = new System.Random();
            Vector2 a, b;
            for(int i = 0; i < RANDOM_ITERATIONS; i++) {
                a = new Vector2((float) (r.NextDouble()*TESTING_SPACE), (float) (r.NextDouble()*TESTING_SPACE));
                b = new Vector2((float) (r.NextDouble()*TESTING_SPACE), (float) (r.NextDouble()*TESTING_SPACE));
                Assert.AreEqual(Geometry.EuclideanDistance(a, b), Geometry.RealMahalanobisDistance(a, b, m), DELTA);
                Assert.AreEqual(Geometry.EuclideanDistance(a, b), Geometry.RealMahalanobisDistance(b, a, m), DELTA);
            }
            //For an covariance matrix the mahalanobis distance is weighted:
            m[0, 0] = 3f;
            m[1, 0] = 4f;
            m[0, 1] = 5f;
            m[1, 1] = 6f;
            a = new Vector2(7f, 8f);
            b = new Vector2(9f, 10f);
            Assert.AreEqual(Mathf.Sqrt(72f), Geometry.RealMahalanobisDistance(a, b, m), DELTA);
            Assert.AreEqual(72f, Geometry.SquaredMahalanobisDistance(a, b, m), DELTA);
        }
    }
}
