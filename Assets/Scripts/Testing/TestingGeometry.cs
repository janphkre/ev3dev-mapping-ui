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
            Assert.AreEqual(153.08d, Geometry.EuclideanDistance(new Vector2(48.5f, -11.9f), new Vector2(-66.8f, 88.8f)), 0.01d);

            //Squared Vector2, Vector2
            Assert.AreEqual(0.0d, Geometry.SquaredEuclideanDistance(Vector2.zero, Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.SquaredEuclideanDistance(new Vector2(0.0f, 1.0f), Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.SquaredEuclideanDistance(new Vector2(1.0f, 0.0f), Vector2.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.SquaredEuclideanDistance(Vector2.zero, new Vector2(1.0f, 0.0f)), DELTA);
            Assert.AreEqual(1.0d, Geometry.SquaredEuclideanDistance(Vector2.zero, new Vector2(0.0f, 1.0f)), DELTA);
            Assert.AreEqual(2.0d, Geometry.SquaredEuclideanDistance(new Vector2(1.0f, 1.0f), Vector2.zero), DELTA);
            Assert.AreEqual(2.0d, Geometry.SquaredEuclideanDistance(Vector2.zero, new Vector2(1.0f, 1.0f)), DELTA);

            Assert.AreEqual(149.0d, Geometry.SquaredEuclideanDistance(new Vector2(10.0f, 10.0f), new Vector2(20.0f, 17.0f)), DELTA);
            Assert.AreEqual(0.0d, Geometry.SquaredEuclideanDistance(new Vector2(23.5f, 23.5f), new Vector2(23.5f, 23.5f)), DELTA);
            Assert.AreEqual(23434.58d, Geometry.SquaredEuclideanDistance(new Vector2(48.5f, -11.9f), new Vector2(-66.8f, 88.8f)), 0.01d);


            //Vector3, Vector3
            /*Assert.AreEqual(0.0d, Geometry.EuclideanDistance(Vector3.zero, Vector3.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector3(0.0f, 1.0f), Vector3.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(new Vector3(1.0f, 0.0f), Vector3.zero), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector3.zero, new Vector3(1.0f, 0.0f)), DELTA);
            Assert.AreEqual(1.0d, Geometry.EuclideanDistance(Vector3.zero, new Vector3(0.0f, 1.0f)), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(new Vector3(1.0f, 1.0f), Vector3.zero), DELTA);
            Assert.AreEqual(Math.Sqrt(2.0d), Geometry.EuclideanDistance(Vector3.zero, new Vector3(1.0f, 1.0f)), DELTA);

            Assert.AreEqual(Math.Sqrt(149.0d), Geometry.EuclideanDistance(new Vector3(10.0f, 10.0f), new Vector3(20.0f, 17.0f)), DELTA);
            Assert.AreEqual(0.0d, Geometry.EuclideanDistance(new Vector3(23.5f, 23.5f), new Vector3(23.5f, 23.5f)), DELTA);
            Assert.AreEqual(153.08, Geometry.EuclideanDistance(new Vector3(48.5f, -11.9f), new Vector3(-66.8f, 88.8f)), 0.01d);*/

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

        [Test]
        [TestOf(typeof(Geometry))]
        public void GeometryTestRadius() {
            Vector2[] f = new Vector2[5];
            f[0] = new Vector2(1f, 2f);
            f[1] = new Vector2(4f, 3f);
            f[2] = new Vector2(9f, 9f);
            f[3] = new Vector2(5f, 6f);
            f[4] = new Vector2(8f, 7f);
            Assert.AreEqual(Mathf.Sqrt(162f), Geometry.Radius(Vector2.zero, f), DELTA);
            Assert.AreEqual(Mathf.Sqrt(313f), Geometry.Radius(new Vector2(-3f, -4f), f), DELTA);
        }

        [Test]
        [TestOf(typeof(Geometry))]
        public void GeometryTestRangeBearing() {
            //Vector2, Vector3
            Assert.AreEqual(Vector2.zero, Geometry.ToRangeBearing(Vector2.zero, Vector3.zero));
            Assert.AreEqual(Vector2.zero, Geometry.ToRangeBearing(new Vector2(1.0f, 1.0f), new Vector3(1.0f, 1.0f, 0.0f)));
            Assert.AreEqual(new Vector2(1.0f, 0.0f), Geometry.ToRangeBearing(new Vector2(1.0f, 0.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(1.0f, Geometry.HALF_CIRCLE), Geometry.ToRangeBearing(new Vector2(-1.0f, 0.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(2.0f, Geometry.RIGHT_ANGLE), Geometry.ToRangeBearing(new Vector2(0.0f, 2.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(3.0f, 0.0f), Geometry.ToRangeBearing(new Vector2(0.0f, 3.0f), new Vector3(0.0f, 0.0f, Geometry.RIGHT_ANGLE)));
            Assert.AreEqual(new Vector2(4.0f, Geometry.RIGHT_ANGLE), Geometry.ToRangeBearing(new Vector2(-3.0f, 0.0f), new Vector3(1.0f, 0.0f, Geometry.RIGHT_ANGLE)));

            //Vector3, Vector3
            Assert.AreEqual(Vector2.zero, Geometry.ToRangeBearing(new Vector3(0.0f, float.NaN, 0.0f), Vector3.zero));
            Assert.AreEqual(Vector2.zero, Geometry.ToRangeBearing(new Vector3(1.0f, float.NaN, 1.0f), new Vector3(1.0f, 1.0f, 0.0f)));
            Assert.AreEqual(new Vector2(1.0f, 0.0f), Geometry.ToRangeBearing(new Vector3(1.0f, float.NaN, 0.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(1.0f, Geometry.HALF_CIRCLE), Geometry.ToRangeBearing(new Vector3(-1.0f, float.NaN, 0.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(2.0f, Geometry.RIGHT_ANGLE), Geometry.ToRangeBearing(new Vector3(0.0f, float.NaN, 2.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(3.0f, 0.0f), Geometry.ToRangeBearing(new Vector3(0.0f, float.NaN, 3.0f), new Vector3(0.0f, 0.0f, Geometry.RIGHT_ANGLE)));
            Assert.AreEqual(new Vector2(4.0f, Geometry.RIGHT_ANGLE), Geometry.ToRangeBearing(new Vector3(-3.0f, float.NaN, 0.0f), new Vector3(1.0f, 0.0f, Geometry.RIGHT_ANGLE)));

            Assert.AreEqual(new Vector2(2.0f, -Geometry.RIGHT_ANGLE), Geometry.ToRangeBearing(new Vector3(0.0f, float.NaN, -2.0f), Vector3.zero));
            Assert.AreEqual(new Vector2(4.0f, Geometry.HALF_CIRCLE + Geometry.RIGHT_ANGLE), Geometry.ToRangeBearing(new Vector3(-3.0f, float.NaN, 0.0f), new Vector3(1.0f, 0.0f, -Geometry.RIGHT_ANGLE)));

            //float, float
            Assert.AreEqual(Vector2.zero, Geometry.FromRangeBearing(0.0f, 0.0f));
            Assert.AreEqual(new Vector2(5.0f,0.0f), Geometry.FromRangeBearing(5.0f, 0.0f));
            Vector2 v = Geometry.FromRangeBearing(5.0f, Geometry.FULL_CIRCLE);
            Assert.AreEqual(5.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.HALF_CIRCLE);
            Assert.AreEqual(-5.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.RIGHT_ANGLE);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(5.0f, v.y, DELTA);
            
            //float, float, Vector3
            v = Geometry.FromRangeBearing(0.0f, 0.0f, Vector3.zero);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, 0.0f, Vector3.zero);
            Assert.AreEqual(5.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.FULL_CIRCLE, Vector3.zero);
            Assert.AreEqual(5.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.HALF_CIRCLE, Vector3.zero);
            Assert.AreEqual(-5.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.RIGHT_ANGLE, Vector3.zero);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(5.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(0.0f, 0.0f, new Vector3(1.0f, 1.0f, 0.0f));
            Assert.AreEqual(1.0f, v.x, DELTA);
            Assert.AreEqual(1.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, 0.0f, new Vector3(1.0f, 1.0f, Geometry.RIGHT_ANGLE));
            Assert.AreEqual(1.0f, v.x, DELTA);
            Assert.AreEqual(6.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.FULL_CIRCLE, new Vector3(1.0f, 1.0f, Geometry.FULL_CIRCLE));
            Assert.AreEqual(6.0f, v.x, DELTA);
            Assert.AreEqual(1.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.HALF_CIRCLE, new Vector3(1.0f, 1.0f, Geometry.RIGHT_ANGLE));
            Assert.AreEqual(1.0f, v.x, DELTA);
            Assert.AreEqual(-4.0f, v.y, DELTA);
            v = Geometry.FromRangeBearing(5.0f, Geometry.RIGHT_ANGLE, new Vector3(1.0f, 1.0f, Geometry.HALF_CIRCLE));
            Assert.AreEqual(1.0f, v.x, DELTA);
            Assert.AreEqual(-4.0f, v.y, DELTA);

			//float, float, PositionData
			var posData = new PositionData();
			Vector3 w = Geometry.FromRangeBearing(0.0f,0.0f, posData);
            Assert.AreEqual(0.0f, w.x, DELTA);
            Assert.AreEqual(0.0f, w.y, DELTA);
			Assert.AreEqual(0.0f, w.z, DELTA);
			w = Geometry.FromRangeBearing(5.0f,0.0f, posData);
            Assert.AreEqual(5.0f, w.x, DELTA);
            Assert.AreEqual(0.0f, w.y, DELTA);
			Assert.AreEqual(0.0f, w.z, DELTA);
			w = Geometry.FromRangeBearing(5.0f, Geometry.FULL_CIRCLE, posData);
            Assert.AreEqual(5.0f, w.x, DELTA);
            Assert.AreEqual(0.0f, w.y, DELTA);
			Assert.AreEqual(0.0f, w.z, DELTA);
            w = Geometry.FromRangeBearing(5.0f, Geometry.HALF_CIRCLE, posData);
            Assert.AreEqual(-5.0f, w.x, DELTA);
            Assert.AreEqual(0.0f, w.y, DELTA);
			Assert.AreEqual(0.0f, w.z, DELTA);
            w = Geometry.FromRangeBearing(5.0f, Geometry.RIGHT_ANGLE, posData);
            Assert.AreEqual(0.0f, w.x, DELTA);
            Assert.AreEqual(0.0f, w.y, DELTA);
			Assert.AreEqual(5.0f, w.z, DELTA);

			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			w = Geometry.FromRangeBearing(0.0f, 0.0f, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(1.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 90f;//degrees
            w = Geometry.FromRangeBearing(5.0f, 0.0f, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(6.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 360f;//degrees
            w = Geometry.FromRangeBearing(5.0f, Geometry.FULL_CIRCLE, posData);
            Assert.AreEqual(6.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(1.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 90f;//degrees
            w = Geometry.FromRangeBearing(5.0f, Geometry.HALF_CIRCLE, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(-4.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 180f;//degrees
            w = Geometry.FromRangeBearing(5.0f, Geometry.RIGHT_ANGLE, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(-4.0f, w.z, DELTA);

			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 0f;//degrees
			w = Geometry.FromRangeBearing(0.0f, 0.0f, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(1.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = -90f;//degrees
            w = Geometry.FromRangeBearing(5.0f, 0.0f, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(-4.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 360f;//degrees
            w = Geometry.FromRangeBearing(5.0f, -Geometry.RIGHT_ANGLE, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(-4.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = -90f;//degrees
            w = Geometry.FromRangeBearing(5.0f, -Geometry.HALF_CIRCLE, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(6.0f, w.z, DELTA);
			posData.position = new Vector3(1.0f, 1.0f, 1.0f);
			posData.heading = 180f;//degrees
            w = Geometry.FromRangeBearing(5.0f, -Geometry.RIGHT_ANGLE, posData);
            Assert.AreEqual(1.0f, w.x, DELTA);
            Assert.AreEqual(1.0f, w.y, DELTA);
			Assert.AreEqual(6.0f, w.z, DELTA);

        }

		[Test]
        [TestOf(typeof(Geometry))]
        public void GeometryTestRotate() {
            Vector2 v = Geometry.Rotate(Vector2.zero, Vector3.zero, 0.0f);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.Rotate(new Vector2(0.0f, -1.0f), Vector3.zero, Geometry.RIGHT_ANGLE);
            Assert.AreEqual(1.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.Rotate(new Vector2(0.0f, -5.0f), Vector3.zero, Geometry.HALF_CIRCLE);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(5.0f, v.y, DELTA);
            v = Geometry.Rotate(new Vector2(0.0f, 5.0f), Vector3.zero, Geometry.FULL_CIRCLE);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(5.0f, v.y, DELTA);
            v = Geometry.Rotate(new Vector2(0.0f, 5.0f), new Vector3(0.0f, 0.0f, Geometry.RIGHT_ANGLE), Geometry.FULL_CIRCLE);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(5.0f, v.y, DELTA);
            v = Geometry.Rotate(new Vector2(0.0f, 5.0f), new Vector3(0.0f, 0.0f, Geometry.RIGHT_ANGLE), Geometry.RIGHT_ANGLE);
            Assert.AreEqual(-5.0f, v.x, DELTA);
            Assert.AreEqual(0.0f, v.y, DELTA);
            v = Geometry.Rotate(new Vector2(0.0f, 5.0f), new Vector3(0.0f, 0.0f, Geometry.RIGHT_ANGLE), Geometry.HALF_CIRCLE);
            Assert.AreEqual(0.0f, v.x, DELTA);
            Assert.AreEqual(-5.0f, v.y, DELTA);
		}
    }
}
