using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

[RequireComponent(typeof(Map3D))]
public class ServerMap : NetworkBehaviour {

    public const int MINIMUM_MERGE_COUNT = SLAMRobot.MAX_MAP_SIZE * 2;
    public const float MAP_HEIGHT = 0.5f;

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

    private class ServerClientItem {
        internal GlobalClientMapMessage clientMap;
        internal Vector3 lastClientPose = Vector3.zero;
        internal RobotPose lastGlobalPose = RobotPose.zero;
        internal bool wasMatched = false;

        internal ServerClientItem(GlobalClientMapMessage clientMap) {
            this.clientMap = clientMap;
        }

        internal int GetFeatureCount() {
            return clientMap.globalStateVector.Count - clientMap.localMapCount;
        }
    }

    public override void OnStartServer() {
        base.OnStartServer();
        //NetworkServer.RegisterHandler((short)MessageType.LocalClientMap, OnLocalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.GlobalClientMap, OnGlobalClientMap);
        NetworkServer.RegisterHandler((short)MessageType.Quit, OnQuitMessage);

        clientMaps = new Dictionary<int, ServerClientItem>();
        pairingLocalization = new PairingLocalization();
        utils = new ISLSJFBase();
        map = GetComponent<Map3D>();

        clientInversedCovarianceCollection = new List<SparseCovarianceMatrix>();
        infoMatrix = new SparseCovarianceMatrix();
        infoVector = new SparseColumn();
        globalStateVector = new List<IFeature>();
        globalStateCollection = new List<List<Feature>>();

        
    }

    /*void OnLocalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved Local Client Map Message from " + netMsg.conn.connectionId);
        LocalClientMap msg = netMsg.ReadMessage<LocalClientMap>();
        ServerClientItem value = null;
        if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
            cloudCount++;
            value = new ();
            clientMaps.Add(netMsg.conn.connectionId, value);
        }
        StartCoroutine(value.ConsumeLocalMap(msg));
    }*/

    void OnGlobalClientMap(NetworkMessage netMsg) {
        Debug.Log("Recieved Map Message from " + netMsg.conn.connectionId);
        GlobalClientMapMessage msg = netMsg.ReadMessage<GlobalClientMapMessage>();
        ServerClientItem value = null;
        lock (clientMaps) {
            if (!clientMaps.TryGetValue(netMsg.conn.connectionId, out value)) {
                value = new ServerClientItem(msg);
                clientMaps.Add(netMsg.conn.connectionId, value);
            } else {
                value.clientMap = msg;
            }
        }
        StartCoroutine("mergeSubCloud", value);
    }

    void OnQuitMessage(NetworkMessage netMsg) {
        Debug.Log("Recieved Quit Message from " + netMsg.conn.connectionId);
        clientMaps.Remove(netMsg.conn.connectionId);//TODO: does the map from that client really have to be deleted? -> it is some sort of knowledge that can still be used by the other robots.
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
                    float maxEstimationDistance = Geometry.EuclideanDistance(Vector3.zero, clientMap.lastGlobalPose.pose);
                    lock (clientMaps) {
                        foreach (KeyValuePair<int, ServerClientItem> pair in clientMaps) {
                            if (!clientMap.wasMatched) continue;
                            var currentDistance = Geometry.EuclideanDistance(pair.Value.lastGlobalPose.pose, clientMap.lastGlobalPose.pose);
                            if (currentDistance > maxEstimationDistance) maxEstimationDistance = currentDistance;
                        }
                    }
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
                //TODO: Display Robots.
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