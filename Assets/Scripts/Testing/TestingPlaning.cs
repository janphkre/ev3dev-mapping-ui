using System;
using ev3devMapping.Society;
using NUnit.Framework;
using UnityEngine;

namespace ev3devMapping.Testing {

    //This script is attached to the "Robot" GameObject.
    public class TestingPlaning : ITesting {

        public GameObject GraphObj;
        public PointCloud laserPointCloud;

        public void Start() {
            GraphTestSampleInput();
        }
        
        public void GraphTestSampleInput() {
            Vector2 v;

            PlaningInputData sampleInput = new PlaningInputData();
            sampleInput.Read();
            GraphObj = Instantiate(GraphObj, SceneManager.DynamicObjects);
            Graph graph = GraphObj.GetComponent<Graph>();
            graph.Feed(sampleInput);

            Assert.IsTrue(graph.HasUnvisitedNodes());
            Assert.IsTrue(graph.UnvisitedNodeCount() > 0);
            Assert.IsTrue(graph.GetNewTarget(out v));
            Assert.AreNotEqual(Vector2.zero, v);
            Assert.AreEqual(null, graph.GetStartPath(new Vector3(1.0f,1.0f,0.0f)).Last);

            laserPointCloud = Instantiate(laserPointCloud, SceneManager.DynamicObjects);
            laserPointCloud.numberOfPoints = sampleInput.Readings.Length;
            laserPointCloud.Awake();
            laserPointCloud.SetVertices(sampleInput.Readings);
            Debug.Log(graph.UnvisitedNodeCount());
            graph.DisplayNodes(graph.UnvisitedNodeCount());
        }
    }
}