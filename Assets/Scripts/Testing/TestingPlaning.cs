using System;
using ev3dev.Society;
using ev3devMapping.Society;
using NUnit.Framework;
using UnityEngine;

namespace ev3devMapping.Testing {

    //This script is attached to the "Settings" GameObject.
    public class TestingPlaning : ITesting {

        public GameObject GraphObj;
        public PointCloud laserPointCloud;

        public void Start() {
            PlaningTestFunnel();
            GraphTestSampleInput();
        }

        public void PlaningTestFunnel() {
            Vector2[] funnelTests = {
                new Vector2(0f, 2.3f),
                new Vector2(0f, 2.1f),
                new Vector2(-0.7f, 2.1f),
                new Vector2(-1.9f, 1.5f),
                new Vector2(-2.1f, 1.0f),
                new Vector2(-2.1f, 0.8f),
                new Vector2(-2.1f, 0.7f),
                new Vector2(-2f, 0.7f),
                Vector2.zero,
                new Vector2(1.0f, 0.0f)
            };
            bool[] funnelResults = {
                true,
                false,
                true,
                true,
                true,
                true,
                true,
                true,
                true,
                true
            };
            MainMenu.Physics = new Physics();
            MainMenu.Physics.turningRadius = 1.19f;
            MainMenu.Physics.wheelbaseMm = 185.0f;
            MainMenu.Physics.Calculate();
            Debug.Log("Turning diameter:" + MainMenu.Physics.turningDiameter + "\n Half wheel base:" + MainMenu.Physics.halfWheelbase);
            for(int i = 0; i < funnelTests.Length; i++) {
                var f = Geometry.ToRangeBearing(funnelTests[i], Vector3.zero);
                Assert.AreEqual(funnelResults[i], ExplorePositionCommand.IsWithinFunnel(f),"["+i+"] " + funnelTests[i] + "\n range bearing: " + f, null);
            }
        }

        public void GraphTestSampleInput() {
            var robot = GameObject.Find("Robot(Clone)").transform;
            PlaningInputData sampleInput = new PlaningInputData();
            sampleInput.Read();

            robot.position = new Vector3(sampleInput.LastPose.x, 0f, sampleInput.LastPose.y);
            robot.rotation = Quaternion.AngleAxis(-sampleInput.LastPose.z * 180f / Geometry.HALF_CIRCLE, Vector3.up);
            GraphObj = Instantiate(GraphObj, robot);
            Graph graph = GraphObj.GetComponent<Graph>();
            graph.Feed(sampleInput);

            laserPointCloud = Instantiate(laserPointCloud, SceneManager.DynamicObjects);
            laserPointCloud.numberOfPoints = sampleInput.Readings.Length;
            laserPointCloud.Awake();
            laserPointCloud.SetVertices(sampleInput.Readings);
            graph.DisplayNodes();

            Assert.IsTrue(graph.HasUnvisitedNodes());
            Debug.Log("Graph - unvisited node count: " + graph.UnvisitedNodeCount());
            Assert.IsTrue(graph.UnvisitedNodeCount() > 0);
            Vector2 v;
            Assert.IsTrue(graph.GetNewTarget(out v));
            Assert.AreNotEqual(Vector2.zero, v);
            Assert.AreEqual(Vector2.zero, graph.GetStartPath(new Vector3(1.0f,1.0f,0.0f)).Last.Value);

            Assert.AreEqual("ev3dev.Society.WaitingCommand", Planing.singleton.GetCurrentTargetString());
            Planing.singleton.StartPlaning();
            Assert.AreEqual("ev3dev.Society.ExploreAreaCommand", Planing.singleton.GetCurrentTargetString());

            MainMenu.Physics.turningRadius = 0.595f;
            MainMenu.Physics.Calculate();
            PositionHistory positionHistory = SceneManager.DynamicObjects.gameObject.AddComponent<PositionHistory>();
            var p = new PositionData {
                heading = 0
            };
            positionHistory.PutThreadSafe(p);
            Planing.singleton.LaserReadings = sampleInput;
            //Expect a NullReferenceException
        }
    }
}
