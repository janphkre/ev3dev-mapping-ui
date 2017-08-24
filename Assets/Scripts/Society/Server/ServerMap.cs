using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

namespace ev3devMapping.Society {

[RequireComponent(typeof(Map3D))]
public class ServerMap : NetworkBehaviour {

    public const int MINIMUM_MERGE_COUNT = SLAMRobot.MAX_MAP_SIZE * 2;
    public const float MAP_HEIGHT = 0.5f;
    public const int FIRST_FUSE_RESET_COUNTER = 20;
    public const int FUSE_RESET_COUNTER = 40;

    //Every connection has its own coordinate system. Therefor the different clouds have to be merged through a filter into a complete map.
    private Dictionary<int, ServerClientItem> clientMaps;
    private PairingLocalization pairingLocalization;
    private ISLSJFBase utils;
    private Map3D map;

    private List<SparseCovarianceMatrix> clientInversedCovarianceCollection;//(P^L)^-1
    private SparseCovarianceMatrix infoMatrix;//I(k)
    private SparseColumn infoVector;//i(k)
    private List<IFeature> globalStateVector;//X^G(k)
    private List<List<Feature>> globalStateCollection;//List of all submaps joined into the global map.

    private object colorLock;
    private int colorCounter = 0;

    private object iterationLock;
    private int iteration = 0;
    
    private class ServerClientItem {
        internal GlobalClientMapMessage clientMap = null;
        internal Vector3 lastClientPose = Vector3.zero;
        internal RobotPose lastGlobalPose = RobotPose.zero;
        internal bool wasMatched = false;

        internal Color color;

        internal ServerClientItem(Color color) {
            this.color = color;
        }

        internal ServerClientItem(GlobalClientMapMessage clientMap) {
            this.clientMap = clientMap;
        }

        internal int GetFeatureCount() {
            return clientMap.globalStateVector.Count - clientMap.localMapCount;
        }
    }

    public override void OnStartServer() {
        base.OnStartServer();

        clientMaps = new Dictionary<int, ServerClientItem>();
        pairingLocalization = new PairingLocalization();
        utils = new ISLSJFBase();
        map = GetComponent<Map3D>();

        clientInversedCovarianceCollection = new List<SparseCovarianceMatrix>();
        infoMatrix = new SparseCovarianceMatrix();
        infoVector = new SparseColumn();
        globalStateVector = new List<IFeature>();
        globalStateCollection = new List<List<Feature>>();

        colorLock = new object();
        iterationLock = new object();

        NetworkServer.RegisterHandler((short)MessageType.ColorRequest, OnColor);
        NetworkServer.RegisterHandler((short)MessageType.GlobalClientMap, OnGlobalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.ClientGraph, OnClientGraph);
        NetworkServer.RegisterHandler((short)MessageType.Quit, OnQuitMessage);
    }

    void OnColor(NetworkMessage netMsg) {
        Debug.Log("Recieved color request message from " + netMsg.conn.connectionId);
        float hue;
        lock (colorLock) {
            hue = (1.0f / 3.0f) * (colorCounter % 3);
            int iteration = colorCounter / 3;
            int iterationTwo = Mathf.NextPowerOfTwo(iteration);            
            Debug.Log("Color " + iteration + ", " + iterationTwo);
            if(iteration != 0) hue += (1 + 2 * (iteration - (iterationTwo / 2))) / (6.0f * iterationTwo);
            colorCounter++;
        }
        Color color = Color.HSVToRGB(hue, 1.0f, 1.0f);
        lock (clientMaps) {
            ServerClientItem value;
            if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
                value = new ServerClientItem(color);
                clientMaps.Add(netMsg.conn.connectionId, value);
            } else {
                value.color = color;
            }
        }
        NetworkServer.SendToClient(netMsg.conn.connectionId, (short) MessageType.Color, new ColorMessage(color));
    }

    void OnGlobalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved map message from " + netMsg.conn.connectionId);
        GlobalClientMapMessage msg = netMsg.ReadMessage<GlobalClientMapMessage>();
        ServerClientItem value;
        lock (clientMaps) {
            if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
                value = new ServerClientItem(msg);
                clientMaps.Add(netMsg.conn.connectionId, value);
            } else {
                value.clientMap = msg;
            }
        }
        lock (iterationLock) {
            if (iteration == FIRST_FUSE_RESET_COUNTER || (iteration > FIRST_FUSE_RESET_COUNTER && iteration % FUSE_RESET_COUNTER == 0)) {
                //Reset global map and fuse all maps again.
                StartCoroutine("mergeAllSubClouds");
            } else {
                StartCoroutine("mergeSubCloud", value);
            }
            iteration++;
        }
        //TODO: display robots!
    }

    void OnClientGraph(NetworkMessage netMsg) {
        Debug.Log("Recieved graph message from " + netMsg.conn.connectionId);
        GraphMessage msg = netMsg.ReadMessage<GraphMessage>();
        RobotPose lastGlobalPose;
        Vector3 lastClientPose;
        lock (clientMaps) {
            ServerClientItem value;
            if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) return;
            if (!value.wasMatched) return;
            lastGlobalPose = value.lastGlobalPose;
            lastClientPose = value.lastClientPose;
        }
        //TODO!
    }

    void OnQuitMessage(NetworkMessage netMsg) {
        Debug.Log("Recieved quit message from " + netMsg.conn.connectionId);
        lock (clientMaps) {
            if (!clientMaps[netMsg.conn.connectionId].wasMatched) clientMaps.Remove(netMsg.conn.connectionId);
        }
    }

    void mergeSubCloud(ServerClientItem clientMap) {
        if (clientMap.GetFeatureCount() < MINIMUM_MERGE_COUNT) return;
        lock(globalStateVector) {
            if (globalStateCollection.Count == 0) {
                clientMap.lastClientPose = clientMap.clientMap.lastPose.pose;
                clientMap.lastGlobalPose = new RobotPose(clientMap.lastClientPose, 0.0f);
                List<Feature> collection = new List<Feature>();
                var clientMapEnumerator = clientMap.clientMap.globalStateVector.GetEnumerator();
                while (clientMapEnumerator.MoveNext()) {
                    if (clientMapEnumerator.Current.IsFeature()) {
                        Feature feat = new Feature(((Feature)clientMapEnumerator.Current).feature, clientMap.lastGlobalPose, globalStateVector.Count);
                        globalStateVector.Add(feat);
                        collection.Add(feat);
                    }
                }
                clientMap.lastGlobalPose.index = globalStateVector.Count;
                globalStateVector.Add(clientMap.lastGlobalPose);
                globalStateCollection.Add(collection);
                clientInversedCovarianceCollection.Add(clientMap.clientMap.infoMatrix);
                utils.computeInfoAddition(RobotPose.zero, collection, clientMap.clientMap.infoMatrix, infoMatrix, infoVector, globalStateVector);
                clientMap.wasMatched = true;
            } else {
                //Data Association:
                Vector2 gridOffset;
                float gridSize;
                if (clientMap.wasMatched) {
                    gridOffset = clientMap.lastGlobalPose.pose + (clientMap.clientMap.lastPose.pose - clientMap.lastClientPose);
                    //TODO:
                    gridSize = 1f;
                } else {
                    gridOffset = new Vector2();
                    float maxEstimationDistance = Geometry.SquaredEuclideanDistance(Vector2.zero, clientMap.lastGlobalPose.pose);
                    lock (clientMaps) {
                        foreach (KeyValuePair<int, ServerClientItem> pair in clientMaps) {
                            if (!clientMap.wasMatched) continue;
                            var currentDistance = Geometry.SquaredEuclideanDistance(pair.Value.lastGlobalPose.pose, clientMap.lastGlobalPose.pose);
                            if (currentDistance > maxEstimationDistance) maxEstimationDistance = currentDistance;
                        }
                    }
                    maxEstimationDistance = Mathf.Sqrt(maxEstimationDistance);
                    if (maxEstimationDistance == 0.0f) maxEstimationDistance = 1.0f;
                    float radius = 0.0f;
                    int featureCount = 0;
                    foreach (IFeature f in globalStateVector) {
                        if (radius < f.Magnitude()) radius = f.Magnitude();
                        if (f.IsFeature()) {
                            featureCount++;
                            gridOffset += ((Feature)f).feature / globalStateVector.Count;
                        }
                    }
                    gridOffset *= (float)globalStateVector.Count / featureCount;
                    gridSize = radius * 2 + SLAMRobot.ROBOT_UNCERTAINTY + SLAMRobot.ESTIMATION_ERROR_RATE * globalStateCollection.Count * maxEstimationDistance;
                    clientMap.wasMatched = true;
                }
                List<int> unmatchedClientFeatures;
                List<int> matchedGlobalFeatures;
                Vector3 match = pairingLocalization.Match(gridOffset, gridSize, clientMap.clientMap.lastPose.pose, new FeatureVectorEnumerator(clientMap.clientMap.globalStateVector), new FeatureEnumerator(globalStateVector), out unmatchedClientFeatures, out matchedGlobalFeatures);
                //TODO: check if the match was successful!!
                pairingLocalization.increaseGridUncertanity();
                var pose = new RobotPose(match, 0.0f);
                //Initialize EIF:
                var globalCollection = utils.OffsetLocalMap(pose, clientMap.clientMap.lastPose.pose, new FeatureVectorArray(clientMap.clientMap.globalStateVector), unmatchedClientFeatures, matchedGlobalFeatures, globalStateVector);
                globalStateCollection.Add(globalCollection);
                infoMatrix.Enlarge(unmatchedClientFeatures.Count + 1);
                //Update EIF:
                clientInversedCovarianceCollection.Add(clientMap.clientMap.infoMatrix);
                utils.computeInfoAddition(clientMap.lastGlobalPose, globalCollection, clientMap.clientMap.infoMatrix, infoMatrix, infoVector, globalStateVector);
                utils.MinimumDegreeReorder(infoMatrix, infoVector, globalStateVector);

                recursiveConverging(clientMap.lastGlobalPose, GlobalClientMap.MAX_SMOOTHING_ITERATIONS);
                clientMap.lastGlobalPose = pose;
                clientMap.lastClientPose = clientMap.clientMap.lastPose.pose;
            }
            if (clientMap.wasMatched) {
                ISLSJFBase.DisplayPoints(new FeatureListVectorEnumerator(globalStateCollection), map, MAP_HEIGHT);
            }
        }
    }

    void mergeAllSubClouds() {
        lock(clientMaps) {
            foreach (KeyValuePair<int, ServerClientItem> pair in clientMaps) {
                if (pair.Value.clientMap != null) {
                    pair.Value.wasMatched = false;
                    mergeSubCloud(pair.Value);
                }
            }
        }
    }

    private int indexSize(int i) {
        return globalStateVector[i].IsFeature() ? 2 : 3;
    }

    private void recursiveConverging(RobotPose previousPose, int maxIterations) {
        //2.3.3) and 2.4.2) Compute the Cholesky Factorization of I(k+1)
        SparseTriangularMatrix choleskyFactorization = utils.ComputeCholesky(infoMatrix, indexSize);
        //2.3.4) and 2.4.3) Recover the global map state estimate X^G(k+1)
        SparseColumn y = choleskyFactorization.solveLowerLeftSparse(infoVector);
        SparseColumn globalStateVectorNew = choleskyFactorization.solveUpperRightSparse(y);
        float changeOfEstimate = 0f;//TODO!
        for (int i = 0; i < globalStateVector.Count; i++) {
            IFeature v = globalStateVector[i];
            Matrix m = globalStateVectorNew[i];
            if (v.IsFeature()) {
                Feature f = (Feature)v;
                f.feature.x = m[0, 0];
                f.feature.y = m[0, 1];
            } else {
                RobotPose r = (RobotPose)v;
                r.pose.x = m[0, 0];
                r.pose.y = m[0, 1];
                r.pose.z = m[0, 2];
            }
        }
        //2.4) Least squares for smoothing if necessary
        if (changeOfEstimate <= GlobalClientMap.CHANGE_OF_ESTIMATE_CUTOFF || maxIterations <= 0) return;
        //2.4.1) Recompute the information matrix and the information vector
        infoMatrix.Clear();
        infoVector.Clear();
        for (int i = 0; i < globalStateVector.Count; i++) utils.computeInfoAddition(previousPose, globalStateCollection[i], clientInversedCovarianceCollection[i], infoMatrix, infoVector, globalStateVector);
        //TODO: clientInversedCovarianceCollection will use a lot of memory. Maybe use clientMaps instead somehow?
        recursiveConverging(previousPose, maxIterations - 1);
    }
}
}