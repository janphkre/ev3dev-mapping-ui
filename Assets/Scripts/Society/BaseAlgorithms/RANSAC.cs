using ev3dev.Society.LeastSquares;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

class RANSAC {

    private const int TRIALS = 250;
    private const float SIDE_SEARCH = 0.1f;
    private const float MAX_MATCH_DISTANCE = 0.28f;
    private const int MIN_MATCH_COUNT = 16;
    private const int MAX_HOLE_COUNT = 6;
    private const float MATCH_SEARCH_SIZE = 0.15f;//percent
    private const float MAX_LINE_DISTANCE = 0.5f;//TODO!

    private System.Random random = new System.Random();
    
    private List<Vector3> unmatchedReadings = new List<Vector3>();
    private HashSet<Vector3> matchedReadings = new HashSet<Vector3>();
    private List<Line> lines = new List<Line>();
    private Linear leastSquares;
    private int firstMatchedIndex = -1;
    private int lastMatchedIndex = -1;
    private int holeCounter = 0;
    
    private class Line {
        internal Linear leastSquares;
        internal Vector2 firstMeasurement;
        internal Vector2 lastMeasurement;

        internal Line(Linear leastSquares, Vector3 firstMeasurement, Vector3 lastMeasurement) {
            this.leastSquares = leastSquares;
            this.firstMeasurement = new Vector2(firstMeasurement.x, firstMeasurement.z);
            this.lastMeasurement = new Vector2(lastMeasurement.x, lastMeasurement.z);
        }
    }

    public List<Vector2> FindCorners(Vector3[] readings) {
        unmatchedReadings.AddRange(readings);
        int trials = 0;
        int additionalSide = (int)((readings.Length * SIDE_SEARCH) / 2f);
        if (additionalSide <= 0) additionalSide++;
        while (unmatchedReadings.Count > MIN_MATCH_COUNT * 2 && TRIALS > trials) {
            firstMatchedIndex = -1;
            lastMatchedIndex = -1;
            int maxSearchSize = (int)(unmatchedReadings.Count * MATCH_SEARCH_SIZE);
            int matchCenter = (random.Next(unmatchedReadings.Count)),
                sampleStart = matchCenter - additionalSide,
                sampleEnd = matchCenter + additionalSide; //assuming that the samples are ordered;
            //Please note that this least squares is not robust against y-parallel lines? Is this still true with the changes applied? At least the x-coordinate sorting is useless in this case
            List<Vector3> leastSquaresReadings;
            if(sampleStart < 0) {
                if(sampleEnd >= unmatchedReadings.Count) {
                    Debug.Log("RANSAC - This should not happen." + unmatchedReadings.Count);
                    leastSquaresReadings = unmatchedReadings.GetRange(0, unmatchedReadings.Count);
                } else {
                    leastSquaresReadings = unmatchedReadings.GetRange(unmatchedReadings.Count + sampleStart, -sampleStart);
                    leastSquaresReadings.AddRange(unmatchedReadings.GetRange(0, sampleEnd));
                }
            } else {
                if(sampleEnd > unmatchedReadings.Count) {
                    leastSquaresReadings = unmatchedReadings.GetRange(sampleStart, unmatchedReadings.Count - sampleStart);
                    leastSquaresReadings.AddRange(unmatchedReadings.GetRange(0, sampleEnd - unmatchedReadings.Count));
                } else {
                    leastSquaresReadings = unmatchedReadings.GetRange(sampleStart, sampleEnd - sampleStart);
                }
            }
            leastSquares = new Linear(leastSquaresReadings);
            sampleStart -= maxSearchSize;
            sampleEnd += maxSearchSize;
            int sampleAdditionalEnd = 0;
            if(sampleEnd > unmatchedReadings.Count) {
                sampleAdditionalEnd = sampleEnd - unmatchedReadings.Count;
                sampleEnd = unmatchedReadings.Count - 1;
            }
            if(sampleStart < sampleAdditionalEnd) sampleStart = sampleAdditionalEnd + 1;
            holeCounter = 0;
            if (sampleStart < 0) {
                for(int i = unmatchedReadings.Count + sampleStart; i < unmatchedReadings.Count; i++) {
                    checkReading(i);
                    if(holeCounter > MAX_HOLE_COUNT) {
                        //The hole in the readings was too big.
                        if(i < matchCenter) {
                            //Empty the matchedReadings.
                            matchedReadings.Clear();
                        } else break;
                    }
                }
                sampleStart = 0;
            }
            for (int i = sampleStart; i < sampleEnd; i++) {
                checkReading(i);
                if(holeCounter > MAX_HOLE_COUNT) {
                    //The hole in the readings was too big.
                    if(i < matchCenter) {
                        //Empty the matchedReadings.
                        matchedReadings.Clear();
                    } else break;
                }
            }
            for(int i = 0; i < sampleAdditionalEnd; i++) {
                checkReading(i);
                if(holeCounter > MAX_HOLE_COUNT) {
                    //The hole in the readings was too big.
                    if(i < matchCenter) {
                        //Empty the matchedReadings.
                        matchedReadings.Clear();
                    } else break;
                }
            }
            if (matchedReadings.Count > MIN_MATCH_COUNT) {
                //Consensus that points form a line
                lines.Add(new Line(new Linear(matchedReadings), unmatchedReadings[firstMatchedIndex], unmatchedReadings[lastMatchedIndex]));
                //Remove matched readings from the unmatched readings array:
                if(firstMatchedIndex > lastMatchedIndex) {
                    unmatchedReadings.RemoveRange(firstMatchedIndex, unmatchedReadings.Count - firstMatchedIndex);
                    unmatchedReadings.RemoveRange(0, lastMatchedIndex + 1);
                } else unmatchedReadings.RemoveRange(firstMatchedIndex, lastMatchedIndex - firstMatchedIndex + 1);
            } else trials++;
            matchedReadings.Clear();
        }
        unmatchedReadings.Clear();
        int colorCounter = 0;
        foreach(Line l in lines) {
            float hue = (1.0f / 3.0f) * (colorCounter % 3);
            int iteration = colorCounter / 3;
            int iterationTwo = Mathf.NextPowerOfTwo(iteration);
            if(iteration != 0) hue += (1 + 2 * (iteration - (iterationTwo / 2))) / (6.0f * iterationTwo);
            colorCounter++;
            Color color = Color.HSVToRGB(hue, 1.0f, 1.0f);
            GameObject o = GameObject.CreatePrimitive(PrimitiveType.Cube);
            o.transform.position = new Vector3(l.firstMeasurement.x, 0.5f, l.firstMeasurement.y);
            o.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            o.GetComponent<MeshRenderer>().material.color = color;
            o = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            o.transform.position = new Vector3(l.lastMeasurement.x, 0.5f, l.lastMeasurement.y);
            o.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            o.GetComponent<MeshRenderer>().material.color = color;
        }
        //Each landmark is the corner between two lines found through RANSAC.
        var landmarks = new List<Vector2>();
        for (int i = 0; i < lines.Count - 1; i++) {
            /*//Get the closest lines for the firstMeasurement and the lastMeasurement:
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
                if (Vector2.Distance(lines[i].firstMeasurement, intersection) < MAX_LINE_DISTANCE
                   && Vector2.Distance(lines[closestFront].lastMeasurement, intersection) < MAX_LINE_DISTANCE) landmarks.Add(intersection);
            } catch (ArithmeticException) {
                Debug.Log("RANSAC selected parallel line as closest line!");
            }
            try {
                Vector2 intersection = getIntersection(lines[i].leastSquares, lines[closestBack].leastSquares);
                if (Vector2.Distance(lines[i].lastMeasurement, intersection) < MAX_LINE_DISTANCE
                   && Vector2.Distance(lines[closestBack].firstMeasurement, intersection) < MAX_LINE_DISTANCE) landmarks.Add(intersection);
            } catch (ArithmeticException) {
                Debug.Log("RANSAC selected parallel line as closest line!");
            }*/
            landmarks.Add(lines[i].firstMeasurement);
            landmarks.Add(lines[i].lastMeasurement);
        }
        return landmarks;
    }

    private void checkReading(int i) {
        Vector3 c = unmatchedReadings[i] - leastSquares.Average;
        float f = Geometry.Multiply(leastSquares.Slope, c) / (leastSquares.Slope.magnitude * c.magnitude);
        float distance = c.magnitude * Mathf.Sqrt(1-f*f);
        if (distance < MAX_MATCH_DISTANCE) {
            holeCounter = 0;
            matchedReadings.Add(unmatchedReadings[i]);
            if (firstMatchedIndex == -1) firstMatchedIndex = i;
            lastMatchedIndex = i;
        } else holeCounter++;
    }

    private Vector2 getIntersection(Linear first, Linear second) {
        float s = ((first.Average.z - second.Average.z) * second.Slope.z - (first.Average.x - second.Average.x) * second.Slope.z) / (second.Slope.z * first.Slope.x - first.Slope.z * second.Slope.x);
        if (float.IsNaN(s)) throw new ArithmeticException();
        return new Vector2(first.Average.x + s * first.Slope.x, first.Average.z + s * first.Slope.z);
    }
}
}