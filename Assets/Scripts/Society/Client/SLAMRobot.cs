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

    public const int MAX_MAP_SIZE = 8;
    //private const float VALIDATION_LAMBDA = ???;
    public const float MAX_VALIDATION_INNOVATION = 0.01f;//meters
    public const float MINIMUM_OBSERVED_COUNT = 3;
    public const float GAUSSIAN_SIGMA = 0.15f;
    public const float NOISE_GAUSSIAN_RANGE = 0.01f;//meters
    public const float NOISE_GAUSSIAN_BEARING = 0.5f;//degree

    public const float ROBOT_UNCERTAINTY = 1f;
    public const float ESTIMATION_ERROR_RATE = 1f;

    public static SLAMRobot singelton;

    private System.Random random;
    private Queue<SLAMInputData> input;
    private RANSAC ransac;
    private NearestNeighbour nearestNeighbour;

    private int featureCount;
    private Vector3 lastPose;
    private Vector3 previousInputPose;
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
        ransac = new RANSAC();
        nearestNeighbour = new NearestNeighbour();

        featureCount = 0;
        lastPose = Vector3.zero;
        previousInputPose = Vector3.zero;
        localMap = new LocalClientMap(random, MAX_MAP_SIZE, Vector3.zero);
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
        var landmarks = ransac.FindCorners(data.Readings);
        //TODO: DISPLAY CORNERS?
        //2) Data Association
        //Find the nearest neighbour in the localMap to the extracted landmarks:
        Vector3 inputPose = data.LastPos.position;
        inputPose.y = inputPose.z;
        inputPose.z = data.LastPos.heading;
        List<int> unmatchedLandmarks;
        List<int> matchedFeatures;
        var inversedCovariance = new DefaultedSparseCovarianceMatrix(!localMap.covariance, new Matrix(2));
        var match = nearestNeighbour.Match(inputPose, landmarks.GetEnumerator(), previousInputPose, lastPose, new CombinedFeatureEnumerator(localMap.points.map.GetEnumerator(), featureCount, observedFeatures.GetEnumerator()), inversedCovariance, ROBOT_UNCERTAINTY + ESTIMATION_ERROR_RATE * featureCount, out unmatchedLandmarks, out matchedFeatures);
        //3) Odometry Update
        float deltaX = match.x - lastPose.x;
        float deltaY = match.y - lastPose.y;
        jacobianA[0, 2] = -deltaY;//actually this is y.
        jacobianA[1, 2] = deltaX;
        float headingDelta = match.z - lastPose.z;
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
        //Vector3 posNoise = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * (data.LastPos.position - lastPose);
        //float headingNoise = RandomExtensions.NextGaussian(random, 0, GAUSSIAN_SIGMA) * headingDelta;
        lastPose = match;
        previousInputPose = inputPose;
        //Updating the first covariance row(transposed):
        localMap.covariance[0, 0] = jacobianA * localMap.covariance[0, 0] * ~jacobianA + noiseQ;
        for(int i = 1; i < localMap.covariance.count; i++) localMap.covariance[0, i] = jacobianA * localMap.covariance[0, i];//May throw an error, transpose the matrices then

        //4) Re-observation
        if(jacobianH.sizeX < 3 + featureCount * 2) jacobianH = new Matrix(3 + featureCount * 2, 2);
        jacobianH[1, 2] = -1;
        var unmatchedEnumerator = unmatchedLandmarks.GetEnumerator();
        var matchedEnumerator = matchedFeatures.GetEnumerator();
        unmatchedEnumerator.MoveNext();
        var unmatchedItem = unmatchedEnumerator.Current;
        for (int i = 0; i < landmarks.Count; i++) {
            if(unmatchedItem == i) {
                if(unmatchedEnumerator.MoveNext()) unmatchedItem = unmatchedEnumerator.Current;
                continue;
            }
            matchedEnumerator.MoveNext();
            if (matchedEnumerator.Current < 0) continue;
            float range = (float)Math.Sqrt((landmarks[i].x - lastPose.x) * (landmarks[i].x - lastPose.x)
                + (landmarks[i].y - lastPose.y) * (landmarks[i].y - lastPose.y))
                + vr;//???
            jacobianH[0, 0] = -landmarks[i].x / range;
            jacobianH[0, 1] = -landmarks[i].y / range; 
            jacobianH[1, 0] = landmarks[i].y / range * range;
            jacobianH[1, 1] = landmarks[i].x / range * range;
            int l = matchedEnumerator.Current * 2 + 3;
            jacobianH[l, l] = -jacobianH[0, 0];
            jacobianH[l, l + 1] = -jacobianH[0, 1];
            jacobianH[l + 1, l] = -jacobianH[1, 0];
            jacobianH[l + 1, l + 1] = -jacobianH[1, 1];

            noiseR[0, 0] = range * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_RANGE);
            noiseR[1, 1] = 1;// TODO: page 36/39 - 1degree error. otherwise : (float) Math.Atan2(landmarks[i].y, landmarks[i].x) * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_BEARING);
            Matrix kalmanGainK = localMap.covariance * ~jacobianH * !(jacobianH * (localMap.covariance * ~jacobianH) + noiseV * noiseR * ~noiseV);
            //Update robot position and landmark positions:
            lastPose.x += kalmanGainK[0, 0];
            lastPose.y += kalmanGainK[1, 0];
            lastPose.z += kalmanGainK[2, 0];
            Matrix landmarkDisplacement = new Matrix(1, 2);//TODO: Range and bearing will be needed
            //landmarkDisplacement[0, 0] = landmarks[i].magnitude - localMap.points.map[associatedFeature[i]].magnitude;
            //landmarkDisplacement[0, 1] = ; //TODO!
            for (int j = 0; j < localMap.covariance.count; j++) {
                localMap.points.map[j].x += kalmanGainK[(j * 2) + 3, 0];
                localMap.points.map[j].y += kalmanGainK[(j * 2) + 4, 0];
            }
        }
        unmatchedEnumerator.Dispose();
        matchedEnumerator.Dispose();
        unmatchedEnumerator = unmatchedLandmarks.GetEnumerator();
        matchedEnumerator = matchedFeatures.GetEnumerator();
        unmatchedEnumerator.MoveNext();
        unmatchedItem = unmatchedEnumerator.Current;
        //5 b) New-observation
        for (int i = 0; i < landmarks.Count; i++) {
            if(i == unmatchedItem) {
                //Feature has not been observed yet.
                observedFeatures.Add(new ObservedFeature(landmarks[i]));
                if (unmatchedEnumerator.MoveNext()) unmatchedItem = unmatchedEnumerator.Current;
                continue;
            }
            matchedEnumerator.MoveNext();
            if (matchedEnumerator.Current < 0) {
                //Feature has not been observed more than MINIMUM_OBSERVED_COUNT but at least once
                int l = -matchedEnumerator.Current - 1;
                if (++observedFeatures[l].observedCount > MINIMUM_OBSERVED_COUNT) {
                    //The feature has been observed multiple times and is now considered a feature in the local map.
                    if (featureCount >= MAX_MAP_SIZE) {
                        //5 a) If the current local map is full, we will create a new one and send the old to the ISLSJF global map and the server
                        LocalClientMap oldLocalMap = localMap;
                        oldLocalMap.points.end = lastPose;
                        StartCoroutine("processLocalMap", oldLocalMap);
                        localMap = new LocalClientMap(random, MAX_MAP_SIZE, lastPose);
                        featureCount = 0;
                        observedFeatures.Clear();
                        for (int j = 0; j < landmarks.Count; j++) observedFeatures.Add(new ObservedFeature(landmarks[i]));
                        return; //As we have cleared the observedFeatures list and added all valid landmarks to it we are done with the current scan.
                    }
                    localMap.points.map[featureCount] = observedFeatures[i].feature;
                    featureCount++;
                }
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
                
                float range = (float)Math.Sqrt((localMap.points.map[i].x - lastPose.x) * (localMap.points.map[i].x - lastPose.x) + (localMap.points.map[i].y - lastPose.y) * (localMap.points.map[i].y - lastPose.y)) + vr;//???
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

    private void processLocalMap(LocalClientMap oldLocalMap) {
        oldLocalMap.points.radius = Geometry.Radius(oldLocalMap.points.end, oldLocalMap.points.map);
        //NetworkManager.singleton.client.SendUnreliable((short)MessageType.LocalClientMap, oldLocalMap);
        globalMap.ConsumeLocalMap(oldLocalMap);
    }

    public bool CheckObservedFeature(ObservedFeature obj) {
        return obj.observedCount > MINIMUM_OBSERVED_COUNT;
    }

    public int GetPointCount() {
        return featureCount;
    }

    public void PostOdometryAndReadings(SLAMInputData data) {
        lock(input) input.Enqueue(data);
    }
}
