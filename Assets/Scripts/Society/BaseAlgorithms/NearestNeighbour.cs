/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * See http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.15.7847       *
 * Data Association in Stochastic Mapping Using the Joint Compatibility Test *
 * by Jose Neira, Juan D. Tardos                                             *
 * Implementation by Jan Phillip Kretzschmar                                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

class NearestNeighbour {

    public const float MAX_MATCH_DISTANCE = 1.0f;

    public Vector3 Match(Vector3 measurementsPose, IEnumerator measurements, Vector3 measurementsStart, Vector3 lastMatch, IEnumerator<Feature> features, ICovarianceMatrix featureInversedCovariance, float estimationError, out List<int> unmatchedMeasurements, out List<int> matchedFeatures) {
        Vector3 match = lastMatch + (measurementsPose - measurementsStart);
        unmatchedMeasurements = new List<int>();
        matchedFeatures = new List<int>();
        int i = 0;
        while (measurements.MoveNext()) {
            var measurementRB = Geometry.ToRangeBearing((Vector2) measurements.Current, measurementsPose);
            var measurementTranslated = Geometry.FromRangeBearing(measurementRB.x, measurementRB.y, match);
            float minimumDistance = float.MaxValue;
            Feature minimumFeature = null;
            while (features.MoveNext()) {
                if (matchedFeatures.Contains(features.Current.index)) continue;
                var currentDistance = Geometry.SquaredMahalanobisDistance(features.Current.feature, measurementTranslated, featureInversedCovariance[features.Current.index, features.Current.index]);
                if (currentDistance < minimumDistance) {
                    minimumDistance = currentDistance;
                    minimumFeature = features.Current;
                }
            }
            features.Reset();
            if (minimumFeature == null || minimumDistance > MAX_MATCH_DISTANCE + estimationError) unmatchedMeasurements.Add(i);
            else {
                matchedFeatures.Add(minimumFeature.index);
                //Update the match:
                var featureRB = Geometry.ToRangeBearing(features.Current.feature, match);
                float factor = 1.0f;
                if (i > 0) {
                    match *= i;
                    factor /= (i + 1.0f);
                }
                match.z += featureRB.y - measurementRB.y;
                var feature = Geometry.FromRangeBearing(featureRB.x, measurementRB.y);
                match.x += feature.x - measurementTranslated.x;
                match.y += feature.y - measurementTranslated.y;
                match *= factor;
                //TODO: is this calculation of the match correct?
            }
            i++;
        }
        //TODO:reconsider matching
        return match;
    }
}
}
