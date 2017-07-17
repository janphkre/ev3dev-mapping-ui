using System;
using System.Collections.Generic;
using UnityEngine;

class RANSAC {

    private const int RANSAC_TRIALS = 5;
    private const float RANSAC_INITIAL = 0.015f;
    private const float RANSAC_DISTANCE = 0.01f;
    private const int RANSAC_MIN_FOUND = 10;
    private const float RANSAC_SEARCH_SIZE = 0.2f;//percent
    private const float RANSAC_MAX_CLOSEST_DISTANCE = 1f;//TODO!

    private System.Random random = new System.Random();
    
    private List<Vector3> unmatchedReadings = new List<Vector3>();
    private HashSet<Vector3> matchedReadings = new HashSet<Vector3>();
    private List<Line> lines = new List<Line>();
    private LeastSquares.Linear leastSquares;
    private int firstMatchedIndex = -1;
    private int lastMatchedIndex = -1;
    

    private class Line {
        internal LeastSquares.Linear leastSquares;
        internal Vector2 firstMeasurement;
        internal Vector2 lastMeasurement;

        internal Line(LeastSquares.Linear leastSquares, Vector3 firstMeasurement, Vector3 lastMeasurement) {
            this.leastSquares = leastSquares;
            this.firstMeasurement = new Vector2(firstMeasurement.x, firstMeasurement.z);
            this.lastMeasurement = new Vector2(lastMeasurement.x, lastMeasurement.z);
        }
    }

    public List<Vector2> FindCorners(Vector3[] readings) {
        
        unmatchedReadings.AddRange(readings);
        int trials = 0;
        int additionalSide = (int)((readings.Length * RANSAC_INITIAL - 1f) / 2f);
        if (additionalSide == 0) additionalSide++;
        while (unmatchedReadings.Count > RANSAC_MIN_FOUND * 2 && RANSAC_TRIALS > trials) {
            int maxSearchSize = (int)(unmatchedReadings.Count * RANSAC_SEARCH_SIZE);
            int sampleStart = (random.Next(unmatchedReadings.Count)),
                sampleEnd; //assuming that the samples are ordered;
            if (sampleStart + additionalSide >= unmatchedReadings.Count) {
                sampleEnd = unmatchedReadings.Count - 1;
                sampleStart = sampleEnd - 2 * additionalSide;
            } else if (sampleStart - additionalSide < 0) {
                sampleStart = 0;
                sampleEnd = 2 * additionalSide;
            } else {
                sampleEnd = sampleStart + additionalSide;
                sampleStart -= additionalSide;
            }
            //Please note that this least squares is not robust against y-parallel lines? Is this still true with the changes applied? At least the x-coordinate sorting is useless in this case.
            leastSquares = new LeastSquares.Linear(unmatchedReadings.GetRange(sampleStart, sampleEnd - sampleStart));
            sampleStart -= maxSearchSize;
            int sampleAdditionalEnd = 0;
            if (sampleEnd >= unmatchedReadings.Count + sampleStart) sampleStart = sampleEnd - (unmatchedReadings.Count - 1);
            else {
                sampleEnd += maxSearchSize;
                if (sampleEnd >= unmatchedReadings.Count) {
                    sampleAdditionalEnd = sampleEnd - unmatchedReadings.Count;
                    sampleEnd = unmatchedReadings.Count - 1;
                }
            }
            if (sampleStart < 0) {
                for(int i = unmatchedReadings.Count + sampleStart; i < unmatchedReadings.Count; i++) checkReading(i);
                sampleStart = 0;
            }
            for (int i = sampleStart; i < sampleEnd; i++) checkReading(i);
            if(sampleAdditionalEnd != 0) {
                if (sampleAdditionalEnd >= sampleStart) sampleAdditionalEnd = sampleStart - 1;
                for(int i = 0; i < sampleAdditionalEnd; i++) checkReading(i);
            }
            if (matchedReadings.Count > RANSAC_MIN_FOUND) {
                //Consensus that points form a line
                lines.Add(new Line(new LeastSquares.Linear(matchedReadings), unmatchedReadings[firstMatchedIndex], unmatchedReadings[lastMatchedIndex]));
                //Remove matched readings from the unmatched readings array:
                if(firstMatchedIndex > lastMatchedIndex) {
                    unmatchedReadings.RemoveRange(firstMatchedIndex, unmatchedReadings.Count - firstMatchedIndex);
                    unmatchedReadings.RemoveRange(0, lastMatchedIndex + 1);
                } else unmatchedReadings.RemoveRange(firstMatchedIndex, lastMatchedIndex - firstMatchedIndex + 1);
            } else trials++;
            matchedReadings.Clear();
        }
        unmatchedReadings.Clear();
        //Each landmark is the corner between two lines found through RANSAC.
        var landmarks = new List<Vector2>();
        for (int i = 0; i < lines.Count - 1; i++) {
            //Get the closest lines for the firstMeasurement and the lastMeasurement:
            int j = i + 1,
                closestFront = j,
                closestBack = j;
            float closestFrontDistance = Vector2.Distance(lines[i].firstMeasurement, lines[j].lastMeasurement);
            float closestBackDistance = Vector2.Distance(lines[i].lastMeasurement, lines[j].firstMeasurement);
            for (j++; j < lines.Count; j++) {
                float currentDistance = Vector2.Distance(lines[i].firstMeasurement, lines[j].lastMeasurement);
                if (closestFrontDistance > currentDistance) {
                    closestFrontDistance = currentDistance;
                    closestFront = j;
                }
                currentDistance = Vector2.Distance(lines[i].lastMeasurement, lines[j].firstMeasurement);
                if (closestBackDistance > currentDistance) {
                    closestBackDistance = currentDistance;
                    closestBack = j;
                }
            }
            //Distance check for the closest line:
            try {
                Vector2 intersection = getIntersection(lines[i].leastSquares, lines[closestFront].leastSquares);
                if (Vector2.Distance(lines[i].firstMeasurement, intersection) < RANSAC_MAX_CLOSEST_DISTANCE
                   && Vector2.Distance(lines[closestFront].lastMeasurement, intersection) < RANSAC_MAX_CLOSEST_DISTANCE) landmarks.Add(intersection);
            } catch (ArithmeticException) {
                Debug.Log("RANSAC selected parallel line as closest line!");
            }
            try {
                Vector2 intersection = getIntersection(lines[i].leastSquares, lines[closestBack].leastSquares);
                if (Vector2.Distance(lines[i].lastMeasurement, intersection) < RANSAC_MAX_CLOSEST_DISTANCE
                   && Vector2.Distance(lines[closestBack].firstMeasurement, intersection) < RANSAC_MAX_CLOSEST_DISTANCE) landmarks.Add(intersection);
            } catch (ArithmeticException) {
                Debug.Log("RANSAC selected parallel line as closest line!");
            }
        }
        lines.Clear();
        return landmarks;
    }

    private void checkReading(int i) {
        float distance1 = Vector3.Cross(leastSquares.Slope, unmatchedReadings[i] - leastSquares.Average).magnitude;
        float distance2 = Vector3.Cross(-leastSquares.Slope, unmatchedReadings[i] - leastSquares.Average).magnitude;//TODO:Necessary?
        if (distance1 < RANSAC_DISTANCE || distance2 < RANSAC_DISTANCE) {
            matchedReadings.Add(unmatchedReadings[i]);
            if (firstMatchedIndex == int.MinValue) firstMatchedIndex = i;
            lastMatchedIndex = i;
        }
    }

    private Vector2 getIntersection(LeastSquares.Linear first, LeastSquares.Linear second) {
        float s = ((first.Average.z - second.Average.z) * second.Slope.z - (first.Average.x - second.Average.x) * second.Slope.z) / (second.Slope.z * first.Slope.x - first.Slope.z * second.Slope.x);
        if (float.IsNaN(s)) throw new ArithmeticException();
        return new Vector2(first.Average.x + s * first.Slope.x, first.Average.z + s * first.Slope.z);
    }
}
