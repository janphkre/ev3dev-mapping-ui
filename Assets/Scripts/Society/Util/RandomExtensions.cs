﻿using System;
using System.Collections;
using UnityEngine;

/*
 * https://bitbucket.org/Superbest/superbest-random
 */
namespace Superbest_random {
    /// <summary>
    /// Some extension methods for <see cref="Random"/> for creating a few more kinds of random stuff.
    /// </summary>
    public static class RandomExtensions {
        /// <summary>
        ///   Generates normally distributed numbers. Each operation makes two Gaussians for the price of one, and apparently they can be cached or something for better performance, but who cares.
        /// </summary>
        /// <param name="r"></param>
        /// <param name = "mu">Mean of the distribution</param>
        /// <param name = "sigma">Standard deviation</param>
        /// <returns></returns>
        public static float NextGaussian(this System.Random r, float mu = 0, float sigma = 1) {
            float u1 = (float) r.NextDouble();
            float u2 = (float) r.NextDouble();

            var rand_std_normal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                                Mathf.Sin(2.0f * Mathf.PI * u2);

            var rand_normal = mu + sigma * rand_std_normal;

            return rand_normal;
        }

        /// <summary>
        ///   Generates values from a triangular distribution.
        /// </summary>
        /// <remarks>
        /// See http://en.wikipedia.org/wiki/Triangular_distribution for a description of the triangular probability distribution and the algorithm for generating one.
        /// </remarks>
        /// <param name="r"></param>
        /// <param name = "a">Minimum</param>
        /// <param name = "b">Maximum</param>
        /// <param name = "c">Mode (most frequent value)</param>
        /// <returns></returns>
        public static double NextTriangular(this System.Random r, double a, double b, double c) {
            var u = r.NextDouble();

            return u < (c - a) / (b - a)
                       ? a + Math.Sqrt(u * (b - a) * (c - a))
                       : b - Math.Sqrt((1 - u) * (b - a) * (b - c));
        }

        /// <summary>
        ///   Equally likely to return true or false. Uses <see cref="Random.Next()"/>.
        /// </summary>
        /// <returns></returns>
        public static bool NextBoolean(this System.Random r) {
            return r.Next(2) > 0;
        }

        /// <summary>
        ///   Shuffles a list in O(n) time by using the Fisher-Yates/Knuth algorithm.
        /// </summary>
        /// <param name="r"></param>
        /// <param name = "list"></param>
        public static void Shuffle(this System.Random r, IList list) {
            for (var i = 0; i < list.Count; i++) {
                var j = r.Next(0, i + 1);

                var temp = list[j];
                list[j] = list[i];
                list[i] = temp;
            }
        }
    }
}