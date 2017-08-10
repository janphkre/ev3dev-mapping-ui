/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * SLAM for Dummies                                                                                                                    *
 * See: https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2005/projects/1aslam_blas_repo.pdf  *
 * Paper by Søren Riisgaard and Morten Rufus Blas                                                                                      *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * An EKF-SLAM algorithm with consistency properties                                                                                   *
 * See: https://aps.arxiv.org/abs/1510.06263v3                                                                                         *
 * Paper by Axel Barrau, Silvère Bonnabel                                                                                              *
 * Implementation by Jan Phillip Kretzschmar                                                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using Superbest_random;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SLAMInputData {

    public Vector3 LastPose;
    public Vector3[] Readings;

    public SLAMInputData() { }

    public SLAMInputData(PositionData lastPos, Vector3[] readings, bool[] invalid, int invalidCount) {
        LastPose = new Vector3(lastPos.position.x, lastPos.position.z, lastPos.heading * Mathf.PI / 180f);
        Readings = new Vector3[readings.Length - invalidCount];
        int count = 0;
        for (int i = 0; i < readings.Length; i++) {
            if(!invalid[i]) Readings[count++] = readings[i];
        }
    }
}

[RequireComponent(typeof(Map3D))]
[RequireComponent(typeof(GlobalClientMap))]
public class SLAMRobot : MonoBehaviour {

    public const int MAX_MAP_SIZE = 8;
    //private const float VALIDATION_LAMBDA = ???;
    public const float MAX_VALIDATION_INNOVATION = 0.01f;//meters
    public const float MINIMUM_OBSERVED_COUNT = 3;
    public const float ODOMETRY_SIGMA = 0.15f;
    public const float NOISE_GAUSSIAN_RANGE = 0.01f;//meters
    public const float NOISE_GAUSSIAN_BEARING = 0.5f;//radiant

    public const float ROBOT_UNCERTAINTY = 1f;
    public const float ESTIMATION_ERROR_RATE = 1f;

    public const float MAP_HEIGHT = 0.5f;

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
    private Map3D map;

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

        globalMap = GetComponent<GlobalClientMap>();
        map = GetComponents<Map3D>()[MainMenu.MAP_SLAM_ROBOT];

        jacobianXR[0, 0] = 1;
        jacobianXR[1, 1] = 1;
    }

    private IEnumerator workerRoutine() {
        while (true) {
            yield return new WaitWhile(() => input.Count == 0);
            SLAM();
        }
    }

    private void SLAM() {
        SLAMInputData data;
        lock(input) data = input.Dequeue();
        if (data.Readings.Length == 0) return;
        //1) Landmark Extraction: RANSAC
        var landmarks = ransac.FindCorners(data.Readings);
        //TODO: DISPLAY CORNERS?
        //2) Data Association
        //Find the nearest neighbour in the localMap to the extracted landmarks:
        List<int> unmatchedLandmarks;
        List<int> matchedFeatures;
        var inversedCovariance = new DefaultedSparseCovarianceMatrix(!localMap.covariance, new Matrix(2));
        var match = nearestNeighbour.Match(data.LastPose, landmarks.GetEnumerator(), previousInputPose, lastPose, new CombinedFeatureEnumerator(localMap.map.GetEnumerator(), featureCount, observedFeatures.GetEnumerator()), inversedCovariance, ROBOT_UNCERTAINTY + ESTIMATION_ERROR_RATE * featureCount, out unmatchedLandmarks, out matchedFeatures);
        //Offset found landmarks by match:
        var matchOffset = match - data.LastPose;
        for(int i = 0; i < landmarks.Count; i++) {
            landmarks[i] += (Vector2)matchOffset;
            Geometry.Rotate(landmarks[i], match, matchOffset.z);
        }
        //3) Odometry Update
        var delta = match - lastPose;
        jacobianA[0, 2] = -delta.y;//actually this is y.
        jacobianA[1, 2] = delta.x;
        noiseQ[0, 0] = RandomExtensions.NextGaussian(random, 0, ODOMETRY_SIGMA) * delta.x * delta.x;
        noiseQ[1, 1] = RandomExtensions.NextGaussian(random, 0, ODOMETRY_SIGMA) * delta.y * delta.y;
        noiseQ[2, 2] = RandomExtensions.NextGaussian(random, 0, ODOMETRY_SIGMA) * delta.z * delta.z;
        lastPose = match;
        previousInputPose = data.LastPose;
        //Updating the first covariance row(transposed):
        localMap.covariance[0, 0] = jacobianA * localMap.covariance[0, 0] * ~jacobianA + noiseQ;
        for (int i = 1; i < localMap.covariance.count; i++) {
            localMap.covariance[0, i] = ~(jacobianA * ~localMap.covariance[0, i]);
        }
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
            Vector2 rangeBearing = Geometry.ToRangeBearing(landmarks[i], lastPose);
            jacobianH[0, 0] = lastPose.x - landmarks[i].x / rangeBearing.x;
            jacobianH[0, 1] = lastPose.y - landmarks[i].y / rangeBearing.x; 
            jacobianH[1, 0] = landmarks[i].y - lastPose.y / rangeBearing.x * rangeBearing.x;
            jacobianH[1, 1] = landmarks[i].x - lastPose.x / rangeBearing.x * rangeBearing.x;
            int l = matchedEnumerator.Current * 2 + 3;
            jacobianH[l, l] = -jacobianH[0, 0];
            jacobianH[l, l + 1] = -jacobianH[0, 1];
            jacobianH[l + 1, l] = -jacobianH[1, 0];
            jacobianH[l + 1, l + 1] = -jacobianH[1, 1];

            noiseR[0, 0] = rangeBearing.x * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_RANGE);
            noiseR[1, 1] = rangeBearing.y * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_BEARING);
            Matrix kalmanGainK = localMap.covariance * ~jacobianH * !(jacobianH * (localMap.covariance * ~jacobianH) + noiseV * noiseR * noiseV);//noiseV would have to be translated when used for the second time, but it is an identity matrix.
            //Update robot position and landmark positions:
            Vector2 landmarkDisplacement = landmarks[i] - localMap.map[matchedEnumerator.Current];
            //TODO: Test! is this landmark Displacement correct? (Or do the points in the local map need to be moved by the match as well?)
            lastPose.x += kalmanGainK[0, 0] * landmarkDisplacement.x;
            lastPose.y += kalmanGainK[1, 0] * landmarkDisplacement.x;
            lastPose.z += kalmanGainK[2, 0] * landmarkDisplacement.y;
            localMap.map[matchedEnumerator.Current].x += kalmanGainK[(matchedEnumerator.Current * 2) + 3, 0] * landmarkDisplacement.x;
            localMap.map[matchedEnumerator.Current].y += kalmanGainK[(matchedEnumerator.Current * 2) + 4, 0] * landmarkDisplacement.y;
            //TODO: update covariance: is this correct?
            Matrix identity = new Matrix(localMap.covariance.count);
            localMap.covariance = (identity - kalmanGainK * jacobianH) * localMap.covariance;
        }
        unmatchedEnumerator.Dispose();
        matchedEnumerator.Dispose();
        unmatchedEnumerator = unmatchedLandmarks.GetEnumerator();
        matchedEnumerator = matchedFeatures.GetEnumerator();
        unmatchedEnumerator.MoveNext();
        unmatchedItem = unmatchedEnumerator.Current;
        //5 b) New-observation
        var newFeatures = new List<int>();
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
                        oldLocalMap.end = lastPose;
                        StartCoroutine("processLocalMap", oldLocalMap);
                        localMap = new LocalClientMap(random, MAX_MAP_SIZE, lastPose);
                        featureCount = 0;
                        observedFeatures.Clear();
                        for (int j = 0; j < landmarks.Count; j++) observedFeatures.Add(new ObservedFeature(landmarks[i]));
                        return; //As we have cleared the observedFeatures list and added all valid landmarks to it we are done with the current scan.
                    }
                    localMap[featureCount] = observedFeatures[l].feature;
                    newFeatures.Add(i);
                    featureCount++;
                }
            }
        }
        //Remove all features from the observedFeatures list which have passed MINIMUM_OBSERVED_COUNT
        int k = observedFeatures.RemoveAll((ObservedFeature obj) => obj.observedCount > MINIMUM_OBSERVED_COUNT);
        if (k > 0) {
            localMap.covariance.Enlarge(k);
            var newFeaturesEnumerator = newFeatures.GetEnumerator();
            newFeaturesEnumerator.MoveNext();
            for (int i = localMap.covariance.count - k; i < localMap.covariance.count; i++) {
                jacobianXR[0, 2] = -delta.y;
                jacobianXR[0, 2] = delta.x;

                Vector2 rangeBearing = Geometry.ToRangeBearing(landmarks[newFeaturesEnumerator.Current], lastPose);
                jacobianZ[0, 0] = Mathf.Cos(match.z+rangeBearing.y);
                jacobianZ[1, 0] = Mathf.Sin(match.z+rangeBearing.y);
                jacobianZ[0, 1] = -delta.z * jacobianZ[1, 0];//TODO: what is deltaT supposed to be?
                jacobianZ[1, 1] = delta.z * jacobianZ[0, 0];
                noiseR[0, 0] = rangeBearing.x * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_RANGE);
                noiseR[1, 1] = rangeBearing.y * RandomExtensions.NextGaussian(random, 0, NOISE_GAUSSIAN_BEARING);
                //Calculate the landmark covariance:
                localMap.covariance[i, i] = jacobianXR * localMap.covariance[0, 0] * ~jacobianXR + jacobianZ * noiseR * ~jacobianZ; //TODO: THIS WONT WORK? WHY SHOULD THE WHOLE(!) COVARIANCE MATRIX BE USED FOR THE COVARIANCE OF THE NEW LANDMARK (Cell C)
                //Calculate the robot - landmark covariance:
                localMap.covariance[i, 0] = ~(localMap.covariance[0, 0] * ~jacobianXR);
                //Calculate the landmark - landmark covariance:
                for (int j = 1; j < i; j++) localMap.covariance[j, i] = jacobianXR * localMap.covariance[j, 0];
                newFeaturesEnumerator.MoveNext();
            }
        }
        ISLSJFBase.DisplayPoints(localMap.map.GetEnumerator(), map, MAP_HEIGHT);
    }

    private IEnumerator processLocalMap(LocalClientMap oldLocalMap) {
        oldLocalMap.radius = Geometry.Radius(oldLocalMap.end, oldLocalMap.map);
        //NetworkManager.singleton.client.SendUnreliable((short)MessageType.LocalClientMap, oldLocalMap);
        globalMap.ConsumeLocalMap(oldLocalMap);
        yield return null;
    }

    public int GetPointCount() {
        return featureCount;
    }

    public void PostOdometryAndReadings(SLAMInputData data) {
        lock(input) input.Enqueue(data);
    }
}
