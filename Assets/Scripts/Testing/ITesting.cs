﻿using UnityEngine;
namespace ev3devMapping.Testing {
    public abstract class ITesting: MonoBehaviour {
        public static double DELTA = 0.0001d;
        public static double DELTA_2 = 0.01d;

        public static double TESTING_SPACE = 1000000.0d;
        public static int RANDOM_ITERATIONS = 10000;
        public static int SPARSE_ITERATIONS = 1000;
    }
}
