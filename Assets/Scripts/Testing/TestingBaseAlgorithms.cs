using System;
using ev3devMapping.Society;
using NUnit.Framework;
using UnityEngine;

namespace ev3devMapping.Testing {

    //This script is attached to the "Settings" GameObject.
    public class TestingBaseAlgorithms : ITesting {

        public void Start() {
            TestRansac();
        }

        public void TestRansac() {
            PlaningInputData p = new PlaningInputData();
            p.Read();
            RANSAC r = new RANSAC();
            var result = r.FindCorners(p.Readings);
            Assert.AreNotEqual(0, result.Count);
        }
    }
}