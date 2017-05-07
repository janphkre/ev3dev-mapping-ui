/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SLAM for Dummies                                                                                                                    *
 * See: https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2005/projects/1aslam_blas_repo.pdf  *
 * Paper by Søren Riisgaard and Morten Rufus Blas                                                                                      *
 * Implementation by Jan Phillip Kretzschmar                                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using Superbest_random;

[Serializable]
public class SLAMInputData {
    public PositionData FirstPos;
    public PositionData LastPos;
    public Vector3[] Readings;

    public SLAMInputData() { }

    public SLAMInputData(PositionData firstPos, PositionData lastPos, Vector3[] readings, bool[] invalid, int invalidCount) {
        FirstPos = firstPos;
        LastPos = lastPos;
        Readings = new Vector3[readings.Length - invalidCount];
        int count = 0;
        for (int i = 0; i < Readings.Length; i++) {
            if(!invalid[i]) Readings[count++] = readings[i];
        }
    }
}

public class SLAMRobot : NetworkBehaviour {

    private const int RANSAC_TRIALS = 5;
    private const float RANSAC_INITIAL = 0.015f;
    private const float RANSAC_DISTANCE = 0.01f;
    private const float RANSAC_MIN_FOUND = RANSAC_INITIAL * 2f;
    private const int MAX_MAP_SIZE = 32;
    //private const float VALIDATION_LAMBDA = ???;
    private const float MAX_VALIDATION_INNOVATION = 0.01f;//meters
    private const float MINIMUM_OBSERVED_COUNT = 3;
    private const float GAUSSIAN_SIGMA = 0.15f;
    private const float NOISE_GAUSSIAN_RANGE = 0.01f;//meters
    private const float NOISE_GAUSSIAN_BEARING = 0.5f;//degree

    public static SLAMRobot singelton;

    private System.Random random;
    private Queue<SLAMInputData> input;

    private PositionData lastPosition;
    private LocalClientMap localMap;
    private List<ObservedFeature> observedFeatures;
    //Matrices:
    private Matrix jacobianA;
    private Matrix jacobianH;
    private Matrix jacobianXR;
    private Matrix jacobianZ;
    private Matrix noiseQ;
    private Matrix noiseR;
    private Matrix noiseV;

    private GlobalClientMap globalMap;

    public void Awake() {
        singelton = this;

        random = new System.Random();
        input = new Queue<SLAMInputData>();

        lastPosition = new PositionData();
        lastPosition.position = Vector3.zero;
        lastPosition.heading = 0.0f;
        lastPosition.timestamp = 0L;
        localMap = new LocalClientMap(MAX_MAP_SIZE, Vector3.zero);
        observedFeatures = new List<ObservedFeature>();

        jacobianA = new Matrix(3);
        jacobianH = new Matrix(3, 3);
        jacobianXR = new Matrix(2, 3);
        jacobianZ = new Matrix(2, 2);
        noiseQ = new Matrix(3, 3);
        noiseR = new Matrix(2, 2);
        noiseV = new Matrix(2);
        globalMap = new GlobalClientMap();

        jacobianXR[0, 0] = 1;
        jacobianXR[1, 1] = 1;
    }

    public Vector3[] Range(Vector3[] a, int start, int count) {
        Vector3[] result = new Vector3[count];
        for(int i=0;i< count; i++) result[i] = a[start + i];
        return result;
    }

    public void Update() {
        SLAMInputData data;
        lock(input) {
            if (input.Count == 0) return;
            data = input.Dequeue();
        }
        if (data.Readings.Length == 0) return;

        //1) Landmark Extraction: RANSAC
        //Each landmark is made out of two 2 dimensional vectors. [x, y] form the start of the obstacle and [z, w] form the end. As x <= z We can safe a comparison when merging obstacles.
        //List<Vector4> landmarks = new List<Vector4>();
        Vector3[] unmatchedReadings = new Vector3[data.Readings.Length];
        int unmatchedCount = data.Readings.Length;
        data.Readings.CopyTo(unmatchedReadings, 0);
        int trials = 0;
        int additionalSide = (int)((data.Readings.Length * RANSAC_INITIAL - 1f) / 2f);
        while (unmatchedCount > Math.Ceiling(RANSAC_MIN_FOUND * data.Readings.Length) && RANSAC_TRIALS > trials++) {
            int sampleStart = (random.Next(unmatchedCount)),
                sampleEnd; //assuming that the samples are ordered;
            if (additionalSide == 0) additionalSide++;
            if (sampleStart + additionalSide >= unmatchedCount) {
                sampleEnd = input.Count - 1;
                sampleStart = sampleEnd - 2 * additionalSide;
            } else if (sampleStart - additionalSide < 0) {
                sampleStart = 0;
                sampleEnd = 2 * additionalSide;
            } else {
                sampleEnd = sampleStart + additionalSide;
                sampleStart -= additionalSide;
            }
            //Please note that this least squares is not robust against y-parallel lines? Is this still true with the changes applied? At least the x-coordinate sorting is useless in this case.
            LeastSquares.Linear leastSquares = new LeastSquares.Linear(Range(unmatchedReadings, sampleStart, sampleEnd - sampleStart));
            List<Vector3> matchedReadings = new List<Vector3>();
            List<int> matchedIndexes = new List<int>();
            for (int i = 0; i < unmatchedCount; i++) {
                float distance1 = Vector3.Cross(leastSquares.Slope, unmatchedReadings[i] - leastSquares.Average).magnitude;
                float distance2 = Vector3.Cross(-leastSquares.Slope, unmatchedReadings[i] - leastSquares.Average).magnitude;//Necessary?
                if (distance1 < RANSAC_DISTANCE || distance2 < RANSAC_DISTANCE) {
                    matchedReadings.Add(unmatchedReadings[i]);
                    matchedIndexes.Add(i);
                }
            }
            if (matchedReadings.Count > Math.Ceiling(RANSAC_MIN_FOUND * data.Readings.Length)) {
                //Consensus that points form a line
                leastSquares = new LeastSquares.Linear(matchedReadings);
                //minimum is the vector with the smallest x coordinate
                //maximum is the vector with the biggest x coordinate
                Vector3 minimum = new Vector3(float.PositiveInfinity, 0.0f),
                        maximum = new Vector3(float.NegativeInfinity, 0.0f);
                foreach(Vector3 vec in matchedReadings) {
                    if (minimum.x > vec.x) minimum = vec;
                    if (maximum.x < vec.x) maximum = vec;
                }
                //We then form a second ray that is orthogonal to the least squares ray and calculate the landmark as the least sqaures ray from the intersection of the two rays at the minimum and maxium.
                Ray firstRay = leastSquares.Ray;
                Ray secondRay = new Ray(minimum, new Vector3(-firstRay.direction.z, 0.0f, firstRay.direction.x));
                float minT = (secondRay.direction.z * (secondRay.origin.x - firstRay.origin.x) + secondRay.direction.x * (firstRay.origin.z - firstRay.origin.z)) / (firstRay.direction.x * secondRay.direction.z - firstRay.direction.z * secondRay.direction.x);
                secondRay.origin = maximum;
                float maxT = (secondRay.direction.z * (secondRay.origin.x - firstRay.origin.x) + secondRay.direction.x * (firstRay.origin.z - firstRay.origin.z)) / (firstRay.direction.x * secondRay.direction.z - firstRay.direction.z * secondRay.direction.x);
                landmarks.Add(new Vector4(firstRay.origin.x + minT * firstRay.direction.x, firstRay.origin.z + minT * firstRay.direction.z, firstRay.origin.x + maxT * firstRay.direction.x, firstRay.origin.z + maxT * firstRay.direction.z));
                //Remove matched readings from the unmatched readings array:
                Vector3[] oldUnmatched = unmatchedReadings;
                unmatchedReadings = new Vector3[unmatchedCount - matchedIndexes.Count];
                for (int i = 0, j = 0; i < unmatchedReadings.Length; j++) {
                    if (!matchedIndexes.Contains(j)) unmatchedReadings[i++] = oldUnmatched[j];
                }
            }
        }
        /**UNUSED: extract the lines as points closest to 0,0,0 on the line
        Vector2[] landmarks = new Vector2[extractedLines.Count];
        int k = 0;
        foreach (Ray ray in extractedLines) {
            Vector3 point = Vector3.Dot(-ray.origin, ray.direction) * ray.direction + ray.origin;
            point.y = 0;
            landmarks[k++] = new Vector2(point.x, point.z);
        }**/


        //TODO: DISPLAY LINES? -> matchedReadings or unmatchedReadings in another color

        //2) Data Association
        //Find the nearest neighbor in the localMap to the extracted landmarks:
        int[] associatedFeature = new int[landmarks.Count];
        for (int i = 0; i < landmarks.Count; i++) {
            associatedFeature[i] = MAX_MAP_SIZE;
            float minDistance = float.PositiveInfinity;
            for (int j = 0; j < localMap.featureCount; j++) {
                float distance = Geometry.Distance(landmarks[i], localMap[j]);
                ///float distance = (float) Math.Sqrt(landmarks[i].x * landmarks[i].x + localMap.points.map[j].x * localMap.points.map[j].x - 2 * landmarks[i].x * localMap.points.map[j].x * Math.Cos(localMap.points.map[j].y - landmarks[i].y));
                if (distance < minDistance) {
                    minDistance = distance;
                    associatedFeature[i] = j;
                }
            }
            for (int j = 0; j < observedFeatures.Count; j++) {
                float distance = Geometry.Distance(landmarks[i], observedFeatures[j].feature);
                ///float distance = (float)Math.Sqrt(landmarks[i].x * landmarks[i].x + observedFeatures[j].feature.x * observedFeatures[j].feature.x - 2 * landmarks[i].x * observedFeatures[j].feature.x * Math.Cos(observedFeatures[j].feature.y - landmarks[i].y));
                if (distance < minDistance) {
                    minDistance = distance;
                    associatedFeature[i] = -j-1;
                }
            }
        }
        //Validation:
        for (int i = 0; i < landmarks.Length; i++) {
            if (associatedFeature[i] != MAX_MAP_SIZE) {
                /**each feature in the database may only be associated to one new feature
                for (int j = i; j < landmarks.Length; j++) {
                    if (associatedFeature[i] == associatedFeature[j]) {
                        associatedFeature[i] = MAX_MAP_SIZE+1;
                        break;
                    }
                }**/
                //-Lambda Formula / EKF Uncertainty
                //TODO!
                //float innovation = Vector2.Distance(landmarks[i], localMap.points.map[associatedFeature[i]]);
                //if (~innovation * !covariance * innovation > VALIDATION_LAMBDA) associatedFeature[i] = MAX_MAP_SIZE;
                if (Geometry.Distance(landmarks[i], localMap.points.map[associatedFeature[i]]) > MAX_VALIDATION_INNOVATION) associatedFeature[i] = MAX_MAP_SIZE;
            }
        }

        //3) Odometry Update
        float deltaX = data.LastPos.position.x - lastPosition.position.x;
        float deltaY = data.LastPos.position.z - lastPosition.position.y;
        jacobianA[0, 2] = -deltaY;//actually this is y.
        jacobianA[1, 2] = deltaX;
        float headingDelta = data.LastPos.heading - lastPosition.heading;
        noiseQ[0, 0] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * jacobianA[1, 2] * jacobianA[1, 2];
        noiseQ[0, 1] = jacobianA[1, 2] * (-jacobianA[0, 2]);
        noiseQ[0, 2] = jacobianA[1, 2] * headingDelta;
        noiseQ[1, 0] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * noiseQ[0, 1];
        noiseQ[1, 1] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * jacobianA[0, 2] * jacobianA[0, 2];
        noiseQ[1, 2] = (-jacobianA[0, 2]) * headingDelta;
        noiseQ[2, 0] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * noiseQ[0, 2];
        noiseQ[2, 1] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * noiseQ[1, 2];
        noiseQ[2, 2] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * headingDelta * headingDelta;
        noiseQ[0, 1] *= RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA);
        noiseQ[0, 2] *= RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA);
        noiseQ[1, 2] *= RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA);
        Vector3 posNoise = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * (data.LastPos.position - lastPosition.position);
        float headingNoise = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * (data.LastPos.heading - lastPosition.heading);
        lastPosition = data.LastPos;
        lastPosition.position.y = lastPosition.position.z;
        //Updating the first covariance row:
        localMap.covariance[0, 0] = jacobianA * localMap.covariance[0, 0] * ~jacobianA + noiseQ;
        for(int i = 1; i < localMap.covariance.count; i++) localMap.covariance[0, i] = jacobianA * localMap.covariance[0, i];

        //4) Re-observation
        if(jacobianH.sizeX < 3 + localMap.featureCount * 2) jacobianH = new Matrix(3 + localMap.featureCount * 2, 2);
        jacobianH[1, 2] = -1;
        for (int i = 0; i < landmarks.Length; i++) {
            //TODO: EVENTUELL ANPASSEN -> LANDMARKS ANDERS VERARBEITEN.
            Vector2 center = Geometry.Center(landmarks[i]);
            if (associatedFeature[i] < 0 || associatedFeature[i] >= MAX_MAP_SIZE) continue;
            jacobianH[0, 0] = -center.x / center.magnitude;
            jacobianH[0, 1] = -center.y / center.magnitude; 
            jacobianH[1, 0] = center.y / center.sqrMagnitude;
            jacobianH[1, 1] = center.x / center.sqrMagnitude;
            int k = associatedFeature[i] * 2 + 3;
            jacobianH[k, k] = -jacobianH[0, 0];
            jacobianH[k, k + 1] = -jacobianH[0, 1];
            jacobianH[k + 1, k] = -jacobianH[1, 0];
            jacobianH[k + 1, k + 1] = -jacobianH[1, 1];

            noiseV[0, 0] = landmarks[i].magnitude * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_RANGE);
            noiseV[1, 1] = (float) Math.Atan2(landmarks[i].y, landmarks[i].x) * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_BEARING);
            Matrix landmarkDifference = new Matrix(1, 2);
            landmarkDifference[0, 0] = landmarks[i].magnitude - localMap.points.map[associatedFeature[i]].magnitude;
            landmarkDifference[0, 1] = ; //TODO!
            Matrix kalmanGainK = localMap.covariance * ~jacobianH * !(jacobianH * (localMap.covariance * ~jacobianH) + noiseV * noiseR * ~noiseV) * landmarkDifference;
            lastPosition.position.x += kalmanGainK[0, 0];
            lastPosition.position.y += kalmanGainK[1, 0];
            lastPosition.heading += kalmanGainK[2, 0];
            for (int j = 0; j < localMap.covariance.count; j++) {
                localMap.points.map[j].x += kalmanGainK[(j*2) + 3, 0];
                localMap.points.map[j].y += kalmanGainK[(j*2) + 4, 0];
            }
        }

        if (localMap.featureCount >= MAX_MAP_SIZE) {
            //5 a) If the current local map is full, we will begin with a new one and send the old to the ISLSJF global map and the server
            LocalClientMap oldLocalMap = localMap;
            oldLocalMap.points.end = lastPosition.position;
            //TODO: is this start correct?
            StartCoroutine(ProcessLocalMap(oldLocalMap));
            localMap = new LocalClientMap(MAX_MAP_SIZE, lastPosition.position);
            observedFeatures.Clear();
            for (int i = 0; i < landmarks.Length; i++) {
                if (associatedFeature[i] > MAX_MAP_SIZE) continue;
                observedFeatures.Add(new ObservedFeature(landmarks[i]));
            }
        } else {
            //5 b) New-observation
            for (int i = 0; i < landmarks.Length; i++) {
                if (associatedFeature[i] < 0) {
                    //Feature has not been observed more than MINIMUM_OBSERVED_COUNT but more than once
                    int j = -associatedFeature[i] - 1;
                    if (++observedFeatures[j].observedCount > MINIMUM_OBSERVED_COUNT) {
                        //The feature has been observed multiple times and is now considered a feature in the local map.
                        if (localMap.featureCount < MAX_MAP_SIZE) {
                            localMap.points.map[localMap.featureCount] = observedFeatures[i].feature;
                            localMap.featureCount++;
                        }
                    }
                } else if (associatedFeature[i] == MAX_MAP_SIZE) {
                    //Feature has not been observed yet.
                    observedFeatures.Add(new ObservedFeature(landmarks[i]));
                }
            }
            //Remove all features from the observedFeatures list which have passed MINIMUM_OBSERVED_COUNT
            k = observedFeatures.RemoveAll(checkObservedFeature);
            if (k > 0) {
                localMap.covariance.Enlarge(k);
                for (int i = localMap.covariance.count - k; i < localMap.covariance.count; i++) {
                    jacobianXR[0, 2] = -deltaY;//eventuell deltaTheata einführen
                    jacobianXR[0, 2] = deltaX;
                    jacobianZ[0, 0] = ;
                    jacobianZ[0, 1] = ;
                    jacobianZ[1, 0] = ;
                    jacobianZ[1, 1] = ;
                    //TODO!
                }
            }
        }
    }

    private IEnumerator<LocalClientMap> ProcessLocalMap(LocalClientMap oldLocalMap) {
        NetworkManager.singleton.client.SendUnreliable((short)MessageType.LocalClientMap, oldLocalMap);
        globalMap.ConsumeLocalMap(oldLocalMap);
        yield return null;
    }

    public bool checkObservedFeature(ObservedFeature obj) {
        return obj.observedCount > MINIMUM_OBSERVED_COUNT;
    }

    public int GetPointCount() {
        return localMap.featureCount;
    }

    public void postOdometryAndReadings(SLAMInputData data) {
        lock(input) input.Enqueue(data);
    }
}
