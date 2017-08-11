/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Iterated SLSJF: A Sparse Local Submap Joining Algorithm with Improved Consistency *
 * See http://www.araa.asn.au/acra/acra2008/papers/pap102s1.pdf                      *
 * Paper by Shoudong Huang, Zhan Wang, Gamini Dissanayake and Udo Frese              *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Building upon:                                                                    *
 * SLSJF: Sparse local submap joining filter for building large-scale maps           *
 * See http://services.eng.uts.edu.au/~sdhuang/SLSJF_IEEE_TRO_final_2008_May_27.pdf  *
 * Paper by Shoudong Huang, Zhan Whang and Gamini Dissanayake                        *
 * Implementation by Jan Phillip Kretzschmar                                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;

namespace ev3devMapping.Society {

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * The local map is created by the SLAM algorithm in SLAMRobot.cs.                             *
 * Every local map (should have /) has the same feature count                                  *
 * as the SLAM algorithm will create a new map every time the feature count reaches a cut off. *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
public class LocalClientMap {
    //The map is formed out of features. Each feature is a line. x,y form the start of the line, z,w the end of the line. x <= z should be valid.
    public Vector2[] map;
    public Vector3 end;
    public float radius;
    public CovarianceMatrix covariance;
    public Vector3 start;

    public LocalClientMap(System.Random random, int size, Vector3 start) {
        covariance = new CovarianceMatrix(random);
        map = new Vector2[size];
        this.start = start;
    }

    public Vector2 this[int i] {
        get { return map[i]; }
        set { map[i] = value; }
    }
}

public class GlobalClientMapMessage : MessageBase {

    public RobotPose lastPose;
    public SparseCovarianceMatrix infoMatrix;//I(k)
    public SparseColumn infoVector;//i(k)
    public IFeatureList globalStateVector;//X^G(k)
    public int localMapCount;

    //Should be used only for the network messaging.
    public GlobalClientMapMessage() { }
    
    public GlobalClientMapMessage(SparseCovarianceMatrix infoMatrix, SparseColumn infoVector, List<IFeature> globalStateVector) {
        this.infoMatrix = infoMatrix;
        this.infoVector = infoVector;
        this.globalStateVector = (IFeatureList) globalStateVector;
    }
}

[RequireComponent(typeof(Map3D))]
public class GlobalClientMap: Behaviour {

    public const float ESTIMATION_ERROR_CUTOFF = 1f;//TODO:Tune
    public const float ESTIMATION_ERROR_RATE = 1f;
    public const float CHANGE_OF_ESTIMATE_CUTOFF = 1f;
    public const int MAX_SMOOTHING_ITERATIONS = 10;
    public const int REORDERING_FREQUENCY = 100; // Reorders every REORDERING_FREQUENCY-th iteration
    public const int SEND_FREQUENCY = 10;
    public const float MAP_HEIGHT = 0.75f;
    private PairingLocalization pairingLocalization = new PairingLocalization();
    private NearestNeighbour nearestNeighbour = new NearestNeighbour();
    private ISLSJFBase utils = new ISLSJFBase();

    private int reorderCounter = 1;
    private bool reorderOverride = false;
    private int sendCounter = 1;

    private RobotPose lastPose = RobotPose.zero;
    private List<SparseCovarianceMatrix> localInversedCovarianceCollection = new List<SparseCovarianceMatrix>();//(P^L)^-1
    private SparseTriangularMatrix choleskyFactorization = new SparseTriangularMatrix();//L(k)
    private SparseCovarianceMatrix infoMatrix = new SparseCovarianceMatrix();//I(k)
    private SparseColumn infoVector = new SparseColumn();//i(k)
    private List<IFeature> globalStateVector = new List<IFeature>();//X^G(k)
    private List<List<Feature>> globalStateCollection = new List<List<Feature>>();//List of all submaps joined into the global map.

    public GlobalClientMapMessage message;
    private Map3D map;

    public GlobalClientMap() {
        message = new GlobalClientMapMessage(infoMatrix, infoVector, globalStateVector);
    }

    public void Awake() {        
        map = GetComponents<Map3D>()[MainMenu.MAP_GLOBAL_CLIENT];
    }

    private int indexSize(int i) {
        return globalStateVector[i].IsFeature() ? 2 : 3;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * *
     * Algorithm 1 & 2 of Iterated SLSJF.          *
     * Only completed local maps must be provided. *
     * * * * * * * * * * * * * * * * * * * * * * * */
    public void ConsumeLocalMap(LocalClientMap localMap) {
        if (localMap.map.Length == 0) return;
        if (lastPose == RobotPose.zero) {
            /* * * * * * * * * * * * * * * * * * * * *
             * 1) Set local map 1 as the global map  *
             * * * * * * * * * * * * * * * * * * * * */
            RobotPose pose = new RobotPose(localMap.end, localMap.radius);
            List<Feature> collection = new List<Feature>();
            for (int i = 0; i < localMap.map.Length; i++) {
                Feature feat = new Feature(localMap[i], pose, globalStateVector.Count);
                globalStateVector.Add(feat);
                collection.Add(feat);
            }
            pose.index = globalStateVector.Count;
            lastPose = pose;
            globalStateVector.Add(pose);
            globalStateCollection.Add(collection);
            localInversedCovarianceCollection.Add(!localMap.covariance);
            utils.computeInfoAddition(RobotPose.zero, collection, localInversedCovarianceCollection[0], infoMatrix, infoVector, globalStateVector);
            //choleskyFactorization = utils.ComputeCholesky(infoMatrix, indexSize);
        } else {
            /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
             * 2.1) Data association between local map k+1 and the global map (SLSJF)  *
             * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
            List<int> unmatchedLocalFeatures;
            List<int> matchedGlobalFeatures;
            Vector3 match;
            var prematchedFeatures = new HashSet<int>();
            var start = Vector3.zero;
            float maxEstimationError = 0.0f;
            //2.1.1) Determine the set of potentially overlapping local maps:
            int j = 0;
            foreach (List<Feature> collection in globalStateCollection) {
                RobotPose collectionPose = collection[0].ParentPose();
                float estimationError = SLAMRobot.ROBOT_UNCERTAINTY + ESTIMATION_ERROR_RATE * globalStateCollection.Count * (Geometry.EuclideanDistance(collectionPose.pose, lastPose.pose));
                if ((localMap.end - start).magnitude <= estimationError + localMap.radius + collectionPose.radius) {
                    //2.1.2) Find the set of potentially matched features:
                    for (int i = 0; i < collection.Count; i++) {
                        if (Geometry.EuclideanDistance(localMap.end, collection[i].feature) <= estimationError + localMap.radius) {
                            prematchedFeatures.Add(collection[i].index);//Like this the matchedFeatures should be sorted at all times.
                            if (estimationError > maxEstimationError) maxEstimationError = estimationError;
                        }
                    }
                }
                start = collectionPose.pose;
                j++;
            }
            if (maxEstimationError >= ESTIMATION_ERROR_CUTOFF) {
                reorderOverride = true;//Reorder the info matrix, info vector and global state vector after this step!
                                       //2.1.4) Pair Driven Localization to find the match:
                match = pairingLocalization.Match(lastPose.pose + (localMap.end - localMap.start), localMap.radius + maxEstimationError, localMap.end, localMap.map.GetEnumerator(), new PrematchFeatureEnumerator(globalStateVector, prematchedFeatures), out unmatchedLocalFeatures, out matchedGlobalFeatures);
            } else {
                //2.1.3) Recover the covariance submatrix associated with X^G_(ke) and the potentially matched features:
                SparseColumn q = new SparseColumn(),
                             p;
                SparseColumn columnVector = new SparseColumn();
                SparseCovarianceMatrix subMatrix = new SparseCovarianceMatrix();
                foreach (int feature in prematchedFeatures) {
                    columnVector[feature] = new Matrix(2);
                    q = choleskyFactorization.solveLowerLeftSparse(columnVector);
                    columnVector.Remove(feature);
                    p = choleskyFactorization.solveUpperRightSparse(q);
                    subMatrix.Add(p);
                }
                //Add the last robot position: 
                columnVector = new SparseColumn();
                columnVector[lastPose.index] = new Matrix(3);
                //Solve the sparse linear equations:
                q = choleskyFactorization.solveLowerLeftSparse(columnVector);
                p = choleskyFactorization.solveUpperRightSparse(q);
                subMatrix.Add(p);
                //Remove the unneeded rows from the submatrix:
                subMatrix.Trim(prematchedFeatures, globalStateVector.Count);
                //2.1.4) Nearest Neighbor to find the match:
                //As the actual sensor input is already filter by RANSAC and an reobservation gate, nearest neighbor should be good enough to find the match between global frame and local frame.
                match = nearestNeighbour.Match(localMap.end, localMap.map.GetEnumerator(), localMap.start, lastPose.pose, new PrematchFeatureEnumerator(globalStateVector, prematchedFeatures), subMatrix, maxEstimationError, out unmatchedLocalFeatures, out matchedGlobalFeatures);
            }
            if (unmatchedLocalFeatures.Count == 0) return;//The local map does not contain any new information so we can quit at this point.
            //TODO:(Is this true?)
            pairingLocalization.increaseGridUncertanity();
            /* * * * * * * * * * * * * * * * *
             * 2.2) Initialization using EIF *
             * * * * * * * * * * * * * * * * */
            var pose = new RobotPose(match, localMap.radius);
            var globalCollection = utils.OffsetLocalMap(pose, localMap.end, new VectorArray(localMap.map), unmatchedLocalFeatures, matchedGlobalFeatures, globalStateVector);
            globalStateCollection.Add(globalCollection);
            //Enlarge the info vector, info matrix and cholesky factorization by adding zeros:
            //infoVector is a dictionary, so no enlarging is needed.
            infoMatrix.Enlarge(unmatchedLocalFeatures.Count+1);
            /* * * * * * * * * * * * *
             * 2.3) Update using EIF *
             * * * * * * * * * * * * */
            //2.3.1) Compute the information matrix and vector using EIF
            SparseCovarianceMatrix localMapInversedCovariance = !localMap.covariance;
            localInversedCovarianceCollection.Add(localMapInversedCovariance);
            utils.computeInfoAddition(lastPose, globalCollection, localMapInversedCovariance, infoMatrix, infoVector, globalStateVector);
            //2.3.2) Reorder the global map state vector every 100 steps or after closing large loops
            if (reorderOverride) {
                reorderOverride = false;
                reorderCounter = 0;
                utils.MinimumDegreeReorder(infoMatrix, infoVector, globalStateVector);
            } else {
                reorderCounter = (reorderCounter + 1) % REORDERING_FREQUENCY;
                if (reorderCounter == 0) {
                    utils.MinimumDegreeReorder(infoMatrix, infoVector, globalStateVector);
                }
            }
            //2.3.3) to 2.4) Cholesky, recover global state estimate and least squares smoothing:
            recursiveConverging(lastPose, MAX_SMOOTHING_ITERATIONS);
            lastPose = pose;
            //Send the map to the server:
            sendCounter++;
            sendCounter %= SEND_FREQUENCY;
            if(sendCounter == 0) {
                message.lastPose = lastPose;
                message.localMapCount = globalStateCollection.Count;
                NetworkManager.singleton.client.SendUnreliable((short)MessageType.GlobalClientMap, message);
            }
            //Feed the map into the graph:
            Planing.singleton.GlobalGraph.Feed(globalStateCollection, match - localMap.end, match);
        }
        ISLSJFBase.DisplayPoints(new FeatureListVectorEnumerator(globalStateCollection), map, MAP_HEIGHT);
    }

    private void recursiveConverging(RobotPose lastPose, int maxIterations) {
        //2.3.3) and 2.4.2) Compute the Cholesky Factorization of I(k+1)
        choleskyFactorization = utils.ComputeCholesky(infoMatrix, indexSize);
        //2.3.4) and 2.4.3) Recover the global map state estimate X^G(k+1)
        SparseColumn y = choleskyFactorization.solveLowerLeftSparse(infoVector);
        SparseColumn globalStateVectorNew = choleskyFactorization.solveUpperRightSparse(y);
        float changeOfEstimate = 0f;//TODO!
        for(int i=0;i<globalStateVector.Count;i++) {
            IFeature v = globalStateVector[i];
            Matrix m = globalStateVectorNew[i];
            if (v.IsFeature()) {
                Feature f = (Feature) v;
                f.feature.x = m[0, 0];
                f.feature.y = m[0, 1];
            } else {
                RobotPose r = (RobotPose) v;
                r.pose.x = m[0, 0];
                r.pose.y = m[0, 1];
                r.pose.z = m[0, 2];
            }
        }
        //2.4) Least squares for smoothing if necessary
        if (changeOfEstimate <= CHANGE_OF_ESTIMATE_CUTOFF || maxIterations <= 0) return;
        //2.4.1) Recompute the information matrix and the information vector
        infoMatrix.Clear();
        infoVector.Clear();
        for (int i = 0; i < globalStateVector.Count; i++) utils.computeInfoAddition(lastPose, globalStateCollection[i], localInversedCovarianceCollection[i], infoMatrix, infoVector, globalStateVector);
        recursiveConverging(lastPose, maxIterations - 1);
    }
}
}