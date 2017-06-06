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
        localMap = new LocalClientMap(random, MAX_MAP_SIZE);//TODO: add noise to the initial values in the diagonal (should not be one, see the paper p.31)
        observedFeatures = new List<ObservedFeature>();

        jacobianA = new Matrix(3);
        jacobianH = new Matrix(3, 3);
        jacobianXR = new Matrix(2, 3);
        jacobianZ = new Matrix(2, 2);
        noiseQ = new Matrix(3, 3);
        noiseR = new Matrix(2, 2);
        noiseV = new Matrix(2); //just a identitiy matrix...
        globalMap = new GlobalClientMap();

        jacobianXR[0, 0] = 1;
        jacobianXR[1, 1] = 1;
    }

    public void Update() {
        SLAMInputData data;
        lock(input) {
            if (input.Count == 0) return;
            data = input.Dequeue();
        }
        if (data.Readings.Length == 0) return;
        //1) Landmark Extraction: RANSAC
        //Each landmark is made out of two two dimensional vectors. These two vectors are put into one four dimensional vector: [x, y] for the start of the obstacle and [z, w] for the end. As x <= z is ensured we can safe a comparison when merging obstacles.
        List<Vector4> landmarks = new List<Vector4>();
        Vector3[] unmatchedReadings = new Vector3[data.Readings.Length];
        int unmatchedCount = data.Readings.Length;
        data.Readings.CopyTo(unmatchedReadings, 0);
        int trials = 0;
        int additionalSide = (int)((data.Readings.Length * RANSAC_INITIAL - 1f) / 2f);
        if (additionalSide == 0) additionalSide++;
        while (unmatchedCount > Math.Ceiling(RANSAC_MIN_FOUND * data.Readings.Length) && RANSAC_TRIALS > trials++) {
            int sampleStart = (random.Next(unmatchedCount)),
                sampleEnd; //assuming that the samples are ordered;
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
                //A second ray is formed that is orthogonal to the least squares ray and calculate the landmark as the least squares ray from the intersection of the two rays at the minimum and maxium.
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
        for (int i = 0; i < landmarks.Count; i++) {
            if (associatedFeature[i] != MAX_MAP_SIZE) {
                /*NOT TRUE ANYMORE: each feature in the database may only be associated to one new feature
                for (int j = i; j < landmarks.Length; j++) {
                    if (associatedFeature[i] == associatedFeature[j]) {
                        associatedFeature[i] = MAX_MAP_SIZE+1;
                        break;
                    }
                }*/
                //-Lambda Formula / EKF Uncertainty
                //TODO!
                //float innovation = Vector2.Distance(landmarks[i], localMap.points.map[associatedFeature[i]]);
                //if (~innovation * !covariance * innovation > VALIDATION_LAMBDA) associatedFeature[i] = MAX_MAP_SIZE;
                if (associatedFeature[i] > 0) { if (Geometry.Distance(landmarks[i], localMap.points.map[associatedFeature[i]]) > MAX_VALIDATION_INNOVATION) associatedFeature[i] = MAX_MAP_SIZE; }
                else if (Geometry.Distance(landmarks[i], observedFeatures[-associatedFeature[i]-1].feature) > MAX_VALIDATION_INNOVATION) associatedFeature[i] = MAX_MAP_SIZE;
            }
        }

        //3) Odometry Update
        float deltaX = data.LastPos.position.x - lastPosition.position.x;
        float deltaY = data.LastPos.position.z - lastPosition.position.y;
        jacobianA[0, 2] = -deltaY;//actually this is y.
        jacobianA[1, 2] = deltaX;
        float headingDelta = data.LastPos.heading - lastPosition.heading;
        noiseQ[0, 0] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * deltaX * deltaX;
        noiseQ[0, 1] = deltaX * deltaY;
        noiseQ[0, 2] = deltaX * deltaThrust;
        noiseQ[1, 0] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * noiseQ[0, 1];
        noiseQ[1, 1] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * deltaY * deltaY;
        noiseQ[1, 2] = deltaY * deltaThrust;
        noiseQ[2, 0] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * noiseQ[0, 2];
        noiseQ[2, 1] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * noiseQ[1, 2];
        noiseQ[2, 2] = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * deltaThrust * deltaThrust;
        noiseQ[0, 1] *= RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA);
        noiseQ[0, 2] *= RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA);
        noiseQ[1, 2] *= RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA);
        Vector3 posNoise = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * (data.LastPos.position - lastPosition.position);
        float headingNoise = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * headingDelta;
        lastPosition = data.LastPos;
        lastPosition.position.y = lastPosition.position.z;
        //Updating the first covariance row(transposed):
        localMap.covariance[0, 0] = jacobianA * localMap.covariance[0, 0] * ~jacobianA + noiseQ;
        for(int i = 1; i < localMap.covariance.count; i++) localMap.covariance[0, i] = jacobianA * localMap.covariance[0, i];//May throw an error, transpose the matrices then

        //4) Re-observation
        if(jacobianH.sizeX < 3 + localMap.featureCount * 2) jacobianH = new Matrix(3 + localMap.featureCount * 2, 2);
        jacobianH[1, 2] = -1;
        for (int i = 0; i < landmarks.Count; i++) {
            //TODO: EVENTUELL ANPASSEN -> LANDMARKS ANDERS VERARBEITEN.
            Vector2 center = Geometry.Center(landmarks[i]);
            float range = (float)Math.Sqrt((center.x - lastPosition.position.x) * (center.x - lastPosition.position.x) + (center.y - lastPosition.position.y) * (center.y - lastPosition.position.y)) + vr;//???
            if (associatedFeature[i] < 0 || associatedFeature[i] >= MAX_MAP_SIZE) continue;
            jacobianH[0, 0] = -center.x / range;
            jacobianH[0, 1] = -center.y / range; 
            jacobianH[1, 0] = center.y / range * range;
            jacobianH[1, 1] = center.x / range * range;
            int l = associatedFeature[i] * 2 + 3;
            jacobianH[l, l] = -jacobianH[0, 0];
            jacobianH[l, l + 1] = -jacobianH[0, 1];
            jacobianH[l + 1, l] = -jacobianH[1, 0];
            jacobianH[l + 1, l + 1] = -jacobianH[1, 1];

            noiseR[0, 0] = range * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_RANGE);
            noiseR[1, 1] = 1;// TODO: page 36 - 1degree error. otherwise : (float) Math.Atan2(center.y, center.x) * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_BEARING);
            Matrix kalmanGainK = localMap.covariance * ~jacobianH * !(jacobianH * (localMap.covariance * ~jacobianH) + noiseV * noiseR * ~noiseV);
            //Update robot position and landmark positions:
            lastPosition.position.x += kalmanGainK[0, 0];
            lastPosition.position.y += kalmanGainK[1, 0];
            lastPosition.heading += kalmanGainK[2, 0];
            Matrix landmarkDisplacement = new Matrix(1, 2);//TODO: Range and bearing will be needed
            //landmarkDisplacement[0, 0] = center.magnitude - localMap.points.map[associatedFeature[i]].magnitude;
            //landmarkDisplacement[0, 1] = ; //TODO!
            for (int j = 0; j < localMap.covariance.count; j++) {
                localMap.points.map[j].x += kalmanGainK[(j * 2) + 3, 0];
                localMap.points.map[j].y += kalmanGainK[(j * 2) + 4, 0];
                //TODO: eventuell dimensionalität alles matritzen von 2 auf 4 anheben und beide endpunkte der feature speichern:
                localMap.points.map[j].z += kalmanGainK[(j * 2) + 3, 0];
                localMap.points.map[j].w += kalmanGainK[(j * 2) + 4, 0];
            }
            //TODO: enlarge the re observed feature by the new feature
        }
        
        //5 b) New-observation
        for (int i = 0; i < landmarks.Count; i++) {
            if (associatedFeature[i] < 0) {
                //TODO: enlarge the re observed feature by the new feature
                //Feature has not been observed more than MINIMUM_OBSERVED_COUNT but at least once
                int l = -associatedFeature[i] - 1;
                if (++observedFeatures[l].observedCount > MINIMUM_OBSERVED_COUNT) {
                    //The feature has been observed multiple times and is now considered a feature in the local map.
                    if (localMap.featureCount >= MAX_MAP_SIZE) {
                        //5 a) If the current local map is full, we will create a new one and send the old to the ISLSJF global map and the server
                        LocalClientMap oldLocalMap = localMap;
                        oldLocalMap.points.end = new Vector2(lastPosition.position.x, lastPosition.position.y);
                        //TODO: add sigma to robotPosition (necessary?)
                        //TODO: is this start correct?
                        StartCoroutine(processLocalMap(oldLocalMap));
                        localMap = new LocalClientMap(random, MAX_MAP_SIZE);
                        observedFeatures.Clear();
                        for (int j = 0; j < landmarks.Count; j++) {
                            if (associatedFeature[i] > MAX_MAP_SIZE) continue;
                            observedFeatures.Add(new ObservedFeature(landmarks[i]));
                        }
                        return; //As we have cleared the observedFeatures list and added all valid landmarks to it we are done with the current scan.
                    }
                    localMap.points.map[localMap.featureCount] = observedFeatures[i].feature;
                    localMap.featureCount++;
                }
            } else if (associatedFeature[i] == MAX_MAP_SIZE) {
                //Feature has not been observed yet.
                observedFeatures.Add(new ObservedFeature(landmarks[i]));
            }
        }
        //Remove all features from the observedFeatures list which have passed MINIMUM_OBSERVED_COUNT
        int k = observedFeatures.RemoveAll(CheckObservedFeature);
        if (k > 0) {
            localMap.covariance.Enlarge(k);
            for (int i = localMap.covariance.count - k; i < localMap.covariance.count; i++) {
                jacobianXR[0, 2] = -deltaY;//eventuell deltaTheata einführen
                jacobianXR[0, 2] = deltaX;
                jacobianZ[0, 0] = (float) Math.Cos(data.LastPos.heading);
                jacobianZ[1, 0] = (float) Math.Sin(data.LastPos.heading);
                jacobianZ[0, 1] = -deltaThrust * jacobianZ[1, 0];
                jacobianZ[1, 1] = deltaThrust * jacobianZ[0, 0];

                Vector2 center = Geometry.Center(localMap.points.map[i]);
                float range = (float)Math.Sqrt((center.x - lastPosition.position.x) * (center.x - lastPosition.position.x) + (center.y - lastPosition.position.y) * (center.y - lastPosition.position.y)) + vr;//???
                noiseR[0, 0] = range * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_RANGE);
                noiseR[1, 1] = 1;//TODO: see above (step 4)
                //Calculate the landmark covariance:
                localMap.covariance[i, i] = jacobianXR * localMap.covariance[0, 0] * ~jacobianXR + jacobianZ * noiseR * ~jacobianZ; //TODO: THIS WONT WORK? WHY SHOULD THE WHOLE(!) COVARIANCE MATRIX BE USED FOR THE COVARIANCE OF THE NEW LANDMARK (Cell C)
                //Calculate the robot - landmark covariance:
                localMap.covariance[0, i] = localMap.covariance[0, 0] * ~jacobianXR;
                localMap.covariance[i, 0] = ~localMap.covariance[0, i];
                //Calculate the landmark - landmark covariance:
                for (int j = 1; j < i; j++) {
                    localMap.covariance[j, i] = jacobianXR * ~localMap.covariance[0, j];
                    localMap.covariance[i, j] = ~localMap.covariance[j, i];
                }
            }
        }
    }

    private IEnumerator<LocalClientMap> processLocalMap(LocalClientMap oldLocalMap) {
        oldLocalMap.points.radius = Geometry.Radius(oldLocalMap.points.end, oldLocalMap.points.map);
        NetworkManager.singleton.client.SendUnreliable((short)MessageType.LocalClientMap, oldLocalMap);
        globalMap.ConsumeLocalMap(oldLocalMap);
        yield return null;
    }

    public bool CheckObservedFeature(ObservedFeature obj) {
        return obj.observedCount > MINIMUM_OBSERVED_COUNT;
    }

    public int GetPointCount() {
        return localMap.featureCount;
    }

    public void PostOdometryAndReadings(SLAMInputData data) {
        lock(input) input.Enqueue(data);
    }

    public static Vector3[] Range(Vector3[] a, int start, int count) {
        Vector3[] result = new Vector3[count];
        for (int i = 0; i < count; i++) result[i] = a[start + i];
        return result;
    }
}
