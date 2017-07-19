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

using Superbest_random;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * The local map is created by the SLAM algorithm in SLAMRobot.cs.                             *
 * Every local map (should have /) has the same feature count                                  *
 * as the SLAM algorithm will create a new map every time the feature count reaches a cut off. *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
public class LocalClientMap {

    public FeatureCollection points;
    public CovarianceMatrix covariance;
    public Vector3 start;

    public LocalClientMap(System.Random random, int size, Vector3 start) {
        covariance = new CovarianceMatrix(random);
        points = new FeatureCollection(size);
        this.start = start;
    }

    public Vector2 this[int i] {
        get { return points.map[i]; }
        set { points.map[i] = value; }
    }
}

public class GlobalClientMapMessage : MessageBase {

    public RobotPose lastPose;
    public SparseCovarianceMatrix infoMatrix;//I(k)
    public SparseColumn infoVector;//i(k)
    public List<IFeature> globalStateVector;//X^G(k)
    //public List<List<Feature>> globalStateCollection;//List of all submaps joined into the global map.
    public int localMapCount;

    //Only used for the network messaging.
    public GlobalClientMapMessage() { }
    
    public GlobalClientMapMessage(SparseCovarianceMatrix infoMatrix, SparseColumn infoVector, List<IFeature> globalStateVector, List<List<Feature>> globalStateCollection) {
        this.infoMatrix = infoMatrix;
        this.infoVector = infoVector;
        this.globalStateVector = globalStateVector;
        //this.globalStateCollection = globalStateCollection;
    }
}

public class GlobalClientMap {

    public const float ESTIMATION_ERROR_CUTOFF = 1f;//TODO:Tune
    public const float ESTIMATION_ERROR_RATE = 1f;
    public const float OBSERVATION_NOISE_SIGMA = 1f;
    public const float CHANGE_OF_ESTIMATE_CUTOFF = 1f;
    public const int MAX_SMOOTHING_ITERATIONS = 10;
    public const int REORDERING_FREQUENCY = 100; // Reorders every REORDERING_FREQUENCY-th iteration
    public const int SEND_FREQUENCY = 10;
    
    private System.Random random = new System.Random();
    private PairingLocalization pairingLocalization = new PairingLocalization();
    private NearestNeighbour nearestNeighbour = new NearestNeighbour();
    private int reorderCounter = 1;
    private bool reorderOverride = false;
    private int sendCounter = 0;

    private RobotPose lastPose = RobotPose.zero;
    private List<SparseCovarianceMatrix> localInversedCovarianceCollection = new List<SparseCovarianceMatrix>();//(P^L)^-1
    private SparseTriangularMatrix choleskyFactorization = new SparseTriangularMatrix();//L(k)
    private SparseCovarianceMatrix infoMatrix = new SparseCovarianceMatrix();//I(k)
    private SparseColumn infoVector = new SparseColumn();//i(k)
    private List<IFeature> globalStateVector = new List<IFeature>();//X^G(k)
    private List<List<Feature>> globalStateCollection = new List<List<Feature>>();//List of all submaps joined into the global map.

    public GlobalClientMapMessage message;

    public GlobalClientMap() {
        message = new GlobalClientMapMessage(infoMatrix, infoVector, globalStateVector, globalStateCollection);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * *
     * Algorithm 1 & 2 of Iterated SLSJF.          *
     * Only completed local maps must be provided. *
     * * * * * * * * * * * * * * * * * * * * * * * */
    public void ConsumeLocalMap(LocalClientMap localMap) {
        if (localMap.points.map.Length == 0) return;
        if (lastPose == RobotPose.zero) {
            /* * * * * * * * * * * * * * * * * * * * *
             * 1) Set local map 1 as the global map  *
             * * * * * * * * * * * * * * * * * * * * */
            RobotPose pose = new RobotPose(localMap.points.end, RobotPose.zero, localMap.points.radius);
            List<Feature> collection = new List<Feature>();
            for (int i = 0; i < localMap.points.map.Length; i++) {
                Feature feat = new Feature(localMap.points.map[i], pose, globalStateVector.Count);
                globalStateVector.Add(feat);
                collection.Add(feat);
            }
            pose.index = globalStateVector.Count;
            lastPose = pose;
            globalStateVector.Add(pose);
            globalStateCollection.Add(collection);
            localInversedCovarianceCollection.Add(!localMap.covariance);
            computeInfoAddition(collection, localInversedCovarianceCollection[0]);
            computeCholesky();
        } else {
            /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
             * 2.1) Data association between local map k+1 and the global map (SLSJF)  *
             * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
            List<int> unmatchedLocalFeatures;
            List<int> matchedGlobalFeatures;
            Vector3 match = DataAssociaton(localMap, out unmatchedLocalFeatures, out matchedGlobalFeatures);
            if (unmatchedLocalFeatures.Count == 0) return;//The local map does not contain any new information so we can quit at this point.
            //TODO:(Is this true?)
            pairingLocalization.increaseGridUncertanity();
            /* * * * * * * * * * * * * * * * *
             * 2.2) Initialization using EIF *
             * * * * * * * * * * * * * * * * */
            Vector3 localMatchOffset = new Vector3(match.x - localMap.points.end.x, match.y - localMap.points.end.y, match.z - localMap.points.end.z);
            //Calculate the global positions of all unmatched features
            RobotPose pose = new RobotPose(match, lastPose, localMap.points.radius);
            List<Feature> globalCollection = new List<Feature>();
            int j = 0;
            var matchedGlobalFeaturesEnumerator = matchedGlobalFeatures.GetEnumerator();
            for (int i = 0; i < localMap.points.map.Length; i++) {
                if (i == unmatchedLocalFeatures[j]) {
                    //Move first, rotate second: rotation happens around the moved end pose
                    localMap.points.map[unmatchedLocalFeatures[j]].x += localMatchOffset.x;
                    localMap.points.map[unmatchedLocalFeatures[j]].y += localMatchOffset.y;
                    Feature feat = new Feature(Geometry.Rotate(localMap.points.map[unmatchedLocalFeatures[j++]], match, localMatchOffset.z), pose, globalStateVector.Count);
                    globalStateVector.Add(feat);
                    globalCollection.Add(feat);
                } else {
                    matchedGlobalFeaturesEnumerator.MoveNext();
                    globalCollection.Add((Feature)globalStateVector[matchedGlobalFeaturesEnumerator.Current]);
                }
            }
            matchedGlobalFeaturesEnumerator.Dispose();
            pose.index = globalStateVector.Count;

            globalStateVector.Add(pose);
            globalStateCollection.Add(globalCollection);
            //Enlarge the info vector, info matrix and cholesky factorization by adding zeros:
            //infoVector is a dictionary, so no enlarging is needed.
            infoMatrix.Enlarge(unmatchedLocalFeatures.Count+1);
            choleskyFactorization.Enlarge(unmatchedLocalFeatures.Count+1);
            /* * * * * * * * * * * * *
             * 2.3) Update using EIF *
             * * * * * * * * * * * * */
            //2.3.1) Compute the information matrix and vector using EIF
            SparseCovarianceMatrix localMapInversedCovariance = !localMap.covariance;
            localInversedCovarianceCollection.Add(localMapInversedCovariance);
            computeInfoAddition(globalCollection, localMapInversedCovariance);
            lastPose = pose;
            //2.3.2) Reorder the global map state vector every 100 steps or after closing large loops
            if (reorderOverride) {
                reorderOverride = false;
                reorderCounter = 0;
                minimumDegreeReorder();
            } else {
                reorderCounter = (reorderCounter + 1) % REORDERING_FREQUENCY;
                if (reorderCounter == 0) {
                    minimumDegreeReorder();
                }
            }
            //2.3.3) to 2.4) Cholesky, recover global state estimate and least squares smoothing:
            recursiveConverging(MAX_SMOOTHING_ITERATIONS);

            //Send the map to the server
            sendCounter++;
            sendCounter %= SEND_FREQUENCY;
            if(sendCounter == 0) {
                message.lastPose = lastPose;
                message.localMapCount = globalStateCollection.Count;
                NetworkManager.singleton.client.SendUnreliable((short)MessageType.GlobalClientMap, message);
            }
        }
    }

    private Vector3 DataAssociaton(LocalClientMap localMap, out List<int> unmatchedLocalFeatures, out List<int> matchedGlobalFeatures) {
        var prematchedFeatures = new HashSet<int>();
        var start = Vector3.zero;
        float maxEstimationError = 0.0f;
        //2.1.1) Determine the set of potentially overlapping local maps:
        int j = 0;
        foreach (List<Feature> collection in globalStateCollection) {
            RobotPose pose = collection[0].ParentPose();
            float estimationError = SLAMRobot.ROBOT_UNCERTAINTY + ESTIMATION_ERROR_RATE * globalStateCollection.Count * (Geometry.Distance(pose.pose, lastPose.pose));
            if ((localMap.points.end - start).magnitude <= estimationError + localMap.points.radius + pose.radius) {
                //2.1.2) Find the set of potentially matched features:
                for (int i = 0; i < collection.Count; i++) {
                    if (Geometry.Distance(localMap.points.end, collection[i].feature) <= estimationError + localMap.points.radius) {
                        prematchedFeatures.Add(collection[i].index);//Like this the matchedFeatures should be sorted at all times.
                        if (estimationError > maxEstimationError) maxEstimationError = estimationError;
                    }
                }
            }
            start = pose.pose;
            j++;
        }
        if (maxEstimationError >= ESTIMATION_ERROR_CUTOFF) {
            reorderOverride = true;//Reorder the info matrix, info vector and global state vector after this step!
            //2.1.4) Pair Driven Localization to find the match:
            return pairingLocalization.Match(lastPose.pose + (localMap.points.end - localMap.start), localMap.points.radius + maxEstimationError, localMap.points.end, localMap.points.map.GetEnumerator(), new PrematchFeatureEnumerator(globalStateVector, prematchedFeatures), out unmatchedLocalFeatures, out matchedGlobalFeatures);
        } else {
            //TODO: What are the covariance submatrices needed for?
            /** //2.1.3) Recover the covariance submatrix associated with X^G_(ke) and the potentially matched features:
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
            subMatrix.Trim(prematchedFeatures, globalStateVector.Count);*/
            //2.1.4) Nearest Neighbor to find the match:
            //As the actual sensor input is already filter by RANSAC and an reobservation gate, nearest neighbor should be good enough to find the match between global frame and local frame.
            return nearestNeighbour.Match(localMap.points.end, localMap.points.map.GetEnumerator(), localMap.start, lastPose.pose, new PrematchFeatureEnumerator(globalStateVector, prematchedFeatures), maxEstimationError, out unmatchedLocalFeatures, out matchedGlobalFeatures);
        }
    }
    


    private SparseMatrix computeJacobianH(List<Feature> localMapFeatures) {
        //H_k+1 = relative positions of robot and features in respect to previous global robot position and rotation
        FeatureCollection relativePositionsH = new FeatureCollection(localMapFeatures.Count);
        RobotPose currentPose = localMapFeatures[0].ParentPose();
        RobotPose previousPose = currentPose.PreviousPose();
        for (int i = 0; i < localMapFeatures.Count; i++) {
            Vector2 vec = (localMapFeatures[i]).feature;
            relativePositionsH.map[i] = new Vector2(
                (float)((vec.x - previousPose.pose.x) * Math.Cos(previousPose.pose.z) + (vec.y - previousPose.pose.y) * Math.Sin(previousPose.pose.z)),
                (float)((vec.y - previousPose.pose.y) * Math.Cos(previousPose.pose.z) - (vec.x - previousPose.pose.x) * Math.Sin(previousPose.pose.z))
            );
        }
        relativePositionsH.end = new Vector3(
            (float)((currentPose.pose.x - previousPose.pose.x) * Math.Cos(previousPose.pose.z) + (currentPose.pose.y - previousPose.pose.y) * Math.Sin(previousPose.pose.z)),
            (float)((currentPose.pose.y - previousPose.pose.y) * Math.Cos(previousPose.pose.z) - (currentPose.pose.x - previousPose.pose.x) * Math.Sin(previousPose.pose.z)),
            currentPose.pose.z - previousPose.pose.z
        );

        SparseMatrix jacobianH = new SparseMatrix();
        //TODO:what are the rows and what are the cols???
        jacobianH.Enlarge(localMapFeatures.Count+1);
        //jacobian first three rows:
        Matrix l = new Matrix(3, 3);
        l[0, 0] = (float)-Math.Cos(previousPose.pose.z);
        l[0, 1] = (float)-Math.Sin(previousPose.pose.z);
        l[0, 2] = relativePositionsH.end.y;
        l[1, 0] = -l[0, 1];
        l[1, 1] = l[0, 0];
        l[1, 2] = -relativePositionsH.end.x;
        l[2, 2] = -1f;
        jacobianH[0, previousPose.index] = l;//X^G_ke

        Matrix m = new Matrix(3, 3);
        m[0, 0] = -l[0, 0];
        m[0, 1] = -l[0, 1];
        m[1, 0] = -l[1, 0];
        m[1, 1] = -l[1, 1];
        m[2, 2] = 1f;
        jacobianH[0, currentPose.index] = m;//X^G_k+1e
        //jacobian all other rows:
        for (int i = 0; i < localMapFeatures.Count; i++) {
            Matrix n = new Matrix(2, 3);
            n[0, 0] = l[0, 0];
            n[0, 1] = l[0, 1];
            n[0, 2] = relativePositionsH.map[i].y;
            n[1, 0] = m[0, 1];
            n[1, 1] = l[0, 0];
            n[1, 2] = -relativePositionsH.map[i].x;
            jacobianH[i + 1, previousPose.index] = n;

            Matrix o = new Matrix(2, 2);
            o[0, 0] = m[0, 0];
            o[0, 1] = m[0, 1];
            o[0, 0] = l[0, 1];
            o[1, 1] = o[0, 0];
            jacobianH[i + 1, localMapFeatures[i].index] = o;//According to mathlab code x and y are: jacobianH[localMapFrame, globalMapStateFrame]

        }
        return jacobianH;
    }

    private SparseColumn computeNoiseW(List<Feature> localMapFeatures) {
        //noiseW = w_j = zero mean gaussian "observation noise"
        SparseColumn noiseW = new SparseColumn();
        Matrix m = new Matrix(3, 3);
        noiseW[localMapFeatures[0].ParentPose().index] = m;
        m[0, 0] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
        m[1, 1] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
        m[2, 2] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
        foreach (Feature f in localMapFeatures) {
            m = new Matrix(2, 2);
            noiseW[f.index] = m;
            m[0, 0] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            m[1, 1] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            //TODO: Do [0,1] and [1,0] need noise too?
        }
        return noiseW;
    }

    private void computeInfoAddition(List<Feature> localMapFeatures, SparseCovarianceMatrix localMapInversedCovariance) {
        SparseMatrix jacobianH = computeJacobianH(localMapFeatures);
        //SparseColumn noiseW = computeNoiseW(localMapFeatures);
        
        SparseMatrix infoMatrixAddition = ~jacobianH * localMapInversedCovariance;
        infoVector.Addition(infoMatrixAddition * (/*noiseW +*/ jacobianH * globalStateVector));//TODO:is the noise needed? Or is it actually generated by the sensor?
        infoMatrixAddition *= jacobianH;
        infoMatrix.Addition(infoMatrixAddition);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * See http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.25.8417 *
     * An Approximate Minimum Degree Ordering Algorithm                    *
     * by Rue Camichel Toulouse, P.R. Amestoy, T.A. Davis, I.S. Duff,      *
     * Patrick Amestoy , Timothy A. Davis , Iain , S. Duff                 *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    private void minimumDegreeReorder() {
        //Reorder the information matrix:
        //Variables:
        var v = new List<int>();
        var a = new List<HashSet<int>>();
        var d = new List<int>();
        var super = new List<List<int>>();
        var l = new Dictionary<int, HashSet<int>>();
        //Elements:
        var vBar = new List<int>();
        var e = new List<HashSet<int>>();
        for (int i = 0; i < globalStateVector.Count; i++) {
            v.Add(i);
            var aI = new HashSet<int>();
            var col = infoMatrix.GetColumn(i).val;
            foreach(int j in col.Keys) {
                //For convenience the two/three columns of each column for the coordinates of the robot poses and features are not stripped.
                if(j != i) aI.Add(j);
            }
            a.Add(aI);
            d.Add(0);
            var eI = new HashSet<int>();
            e.Add(eI);
            var superI = new List<int>();
            superI.Add(i);
            super.Add(superI);
        }
        int k = 0;
        while (k < globalStateVector.Count) {
            //Mass elimination:
            int p = 0;
            int vP = v[0];
            for (int i = 1; i < v.Count; i++) {
                if (d[v[i]] < d[vP]) {
                    p = i;
                    vP = v[p];
                }
            }
            var lP = new HashSet<int>();
            lP.UnionWith(a[vP]);
            foreach (int eP in e[vP]) {
                HashSet<int> lE = null;
                if (l.TryGetValue(eP, out lE)) lP.UnionWith(lE);
            }
            lP.ExceptWith(super[vP]);
            lP.TrimExcess();
            l[vP] = lP;
            int[] w = new int[globalStateVector.Count];
            for(int i=0;i<w.Length; i++) w[i] = -1;
            //TODO: make sure that lP only contains SUPERVARIABLES
            foreach (int i in lP) {
                foreach(int eI in e[i]) {
                    if (w[eI] < 0) w[eI] = l[eI].Count;
                    w[eI] -= super[i].Count;
                }
                //Do vP seperately: TODO: is vP actually needed?
                if (w[vP] < 0) w[vP] = l[vP].Count;
                w[vP] -= super[i].Count;
            }
            //Remove p from v:
            v.RemoveAt(p);
            //Agressive elimination:
            foreach (int i in vBar) {
                if (w[i] != 0) continue;
                //Absorb this element from all variables:
                foreach (int vI in v) {
                    if(e[vI].Remove(i)) e[vI].Add(vP);
                }
            }
            foreach (int i in lP) {
                //Remove redundant entries:
                a[i].ExceptWith(lP);
                a[i].ExceptWith(super[vP]);
                //Element absorption:
                e[i].ExceptWith(e[vP]);
                e[i].Add(vP);
                /*//Compute External Degree:
                d[i] = a[i].Count;
                foreach (int superI in super[i]) {
                    if (a[i].Contains(superI)) d[i]--;
                }
                foreach (int eI in e[i]) {
                    d[i] += l[eI].Count;
                    foreach (int superI in super[i]) {
                        if (l[eI].Contains(superI)) d[i]--;
                    }
                }*/
                //Compute Approximate External Degree:
                //First minimum:
                int min = globalStateVector.Count - k;
                //Second minimum:
                int lPICount = lP.Count;
                foreach(int superI in super[i]) {
                    if (lP.Contains(superI)) lPICount--;
                }
                int min2 = d[i] + lPICount;
                if (min > min2) min = min2;
                //Third minimum:
                min2 = a[i].Count;
                foreach (int superI in super[i]) {
                    if (a[i].Contains(superI)) min2--;
                }
                min2 += lPICount;
                foreach (int eI in e[i]) {
                    if (eI == vP) continue;
                    min2 += w[eI] < 0 ? l[eI].Count : w[eI]; //Is l[eI].Count ever used?
                }
                d[i] = min < min2 ? min : min2;
            }
            //Supervariable detection, pairs found:
            var lPArray = new int[lP.Count];
            lP.CopyTo(lPArray);
            HashSet<int> adjI = new HashSet<int>();
            adjI.UnionWith(a[lPArray[0]]);
            adjI.UnionWith(e[lPArray[0]]);
            //TODO: Maybe superI & superJ have to be added.
            adjI.Add(lPArray[0]);
            for (int i = 0; i < lPArray.Length; i++) {
                HashSet<int> adjJ = new HashSet<int>();
                for(int j = lP.Count - 1; j > i; j--) {
                    adjJ.UnionWith(a[lPArray[j]]);
                    adjJ.UnionWith(e[lPArray[j]]);
                    adjJ.Add(lPArray[j]);
                    if (adjI.SetEquals(adjJ)) {
                        //Remove the supervariable j:
                        super[i].AddRange(super[j]);
                        d[lPArray[i]] -= super[j].Count;
                        super[j].Clear();
                        v.Remove(lPArray[j]);
                        a[lPArray[j]].Clear();
                        e[lPArray[j]].Clear();
                    }
                    adjJ.Clear();
                }
                adjI = adjJ;
            }
            //Convert variable p to elemnt p:
            vBar.Add(vP);
            
            a[vP].Clear();
            e[vP].Clear();
            k += super[vP].Count;
        }
        //Sort the information matrix and the information vector accordingly:
        HashSet<int> set = new HashSet<int>();
        Dictionary<int, Matrix> dictVector = new Dictionary<int, Matrix>();
        Dictionary<int, IFeature> dictState = new Dictionary<int, IFeature>();
        Dictionary<int, List<Matrix>> dictMatrix = new Dictionary<int, List<Matrix>>();
        Matrix m;
        var result = vBar.GetEnumerator();
        for (int i = 0; i < vBar.Count; i++) {
            result.MoveNext();
            if (!set.Contains(i)) {
                //The i-th item was not used before; Store it in dictionary:
                set.Add(i);
                dictVector.Add(i, infoVector[i]);
                dictState.Add(i, globalStateVector[i]);
                List<Matrix> list = new List<Matrix>();
                
            }
            if (set.Contains(result.Current)) {
                //The result item can be found in the dictionary (old sorting):
                infoVector[i] = dictVector[result.Current];
                var state = dictState[result.Current];
                state.index = i;
                globalStateVector[i] = state;
                int j = 0;
                foreach (int resJ in vBar) {
                    if ((i > j && result.Current < resJ) || (i < j && result.Current > resJ)) {
                        //The matrix field has to be flipped from the (old) matrix:
                        if (set.Contains(resJ)) m = dictMatrix[resJ][result.Current];
                        else m = infoMatrix[resJ, result.Current];
                        if (m != null) infoMatrix[i, j] = ~m;
                    } else {
                        m = dictMatrix[result.Current][resJ];
                        if (m != null) infoMatrix[i, j] = m;
                    }
                    j++;
                }
            } else {
                //The result item can not be found in the dictionary:
                set.Add(result.Current);
                infoVector[i] = infoVector[result.Current];
                var state = globalStateVector[result.Current];
                state.index = i;
                globalStateVector[i] = state;
                int j = 0;
                foreach (int resJ in vBar) {
                    if ((i > j && result.Current < resJ) || (i < j && result.Current > resJ)) {
                        //The matrix field has to be flipped from the (old) matrix:
                        if (set.Contains(resJ)) m = dictMatrix[resJ][result.Current];
                        else m = infoMatrix[resJ, result.Current];
                        if (m != null) infoMatrix[i, j] = ~m;
                    } else {
                        m = infoMatrix[result.Current, resJ];
                        if (m != null) infoMatrix[i, j] = m;
                    }
                    j++;
                }
            }
        }
        result.Dispose();
    }

    private void recursiveConverging(int maxIterations) {
        //2.3.3) and 2.4.2) Compute the Cholesky Factorization of I(k+1)
        computeCholesky();
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
        for (int i = 0; i < globalStateVector.Count; i++) computeInfoAddition(globalStateCollection[i], localInversedCovarianceCollection[i]);
        recursiveConverging(maxIterations-1);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * See http://www.cs.utexas.edu/~pingali/CS378/2011sp/lectures/chol4.pdf *
     * The Cholesky Factorization                                            *
     * by Keshav Pingali                                                     *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Matrix Computations, 4th Edition                                      *
     * by Gene H. Golub & Charles F. Van Loan                                *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    private void computeCholesky() {
        choleskyFactorization = new SparseTriangularMatrix();
        choleskyFactorization.Enlarge(infoMatrix.ColumnCount());
        IEnumerator<SparseColumn> choleskyEnum = choleskyFactorization.GetColumnEnumerator();
        choleskyEnum.Reset();
        int c = 0;
        foreach (SparseColumn col in infoMatrix.val) {
            if (!choleskyEnum.MoveNext()) throw new IndexOutOfRangeException();//This should never happen, as we created g with the same column count as this matrix.
            if (col[c] == null) continue;
            foreach (KeyValuePair<int, Matrix> pair in col.val) {
                if (pair.Key < c) continue;
                choleskyEnum.Current[pair.Key] = pair.Value.Clone();
            }
            
            c++;
        }
        choleskyEnum.Reset();
        c = 0;
        foreach (SparseColumn col in infoMatrix.val) {
            if (!choleskyEnum.MoveNext()) throw new IndexOutOfRangeException();//This should never happen, as we created the result with the same column count as this matrix.
            if (col[c] == null) continue;
            Matrix choleskyC = choleskyEnum.Current[c];
            for (int i = 0; i < col[c].sizeX; i++) {
                //First element in submatrix c in column i (item is on the diagonal of the matrix):
                for (int k = 0; k < c; k++) {
                    Matrix choleskyK = choleskyFactorization[k, c];
                    if (choleskyK != null) for (int j = 0; j < choleskyK.sizeY; j++) choleskyC[i, i] -= choleskyK[j, i] * choleskyK[j, i];
                }
                for (int j = 0; j < i; j++) choleskyC[i, i] -= choleskyC[j, i] * choleskyC[j, i];
                choleskyC[i, i] = (float)Math.Sqrt(choleskyC[i, i]);
                //No need to continue with the other elements of the column as we would be dividing by zero.
                if (choleskyC[i, i] == 0.0f) continue;
                //Other elements j of submatrix c in column i
                for(int m = i + 1; m < col[c].sizeY; m++) {
                    for (int l = 0; l < c; l++) {
                        Matrix choleskyL = choleskyFactorization[l, c];
                        if (choleskyL != null)
                            for (int j = 0; j < choleskyL.sizeX; j++) choleskyC[i, m] -= choleskyL[j, m] * choleskyL[j, i];
                    }
                    for (int j = 0; j < i; j++) choleskyC[i, m] -= choleskyC[j, m] * choleskyC[j, i];
                    choleskyC[i, m] /= choleskyC[i, i];
                }
                //All elements m of submatrix k in column i
                for (int k = c + 1; k < infoMatrix.ColumnCount(); k++) {
                    Matrix choleskyK = choleskyEnum.Current[k];
                    if (choleskyK == null) {
                        choleskyK = new Matrix(col[c].sizeX, globalStateVector[c].IsFeature() ? 2 : 3);
                        choleskyEnum.Current[k] = choleskyK;
                    }
                    for (int m = 0; m < choleskyK.sizeY; m++) {
                        for (int l = 0; l < c; l++) {
                            Matrix choleskyL = choleskyFactorization[l, c];
                            Matrix choleskyM = choleskyFactorization[l, k];
                            if (choleskyL != null && choleskyM != null)
                                for (int j = 0; j < choleskyL.sizeX; j++) choleskyK[i, m] -= choleskyM[j, m] * choleskyL[j, i];
                        }
                        for (int j = 0; j < i; j++) choleskyK[i, m] -= choleskyK[j, m] * choleskyC[j, i];
                        choleskyK[i, m] /= choleskyC[i, i];
                    }
                }
            }
            for (int i=c;i< infoMatrix.ColumnCount();i++) if (choleskyEnum.Current[i].IsEmpty()) choleskyEnum.Current.Remove(i);
            c++;
        }
        choleskyEnum.Dispose();
    }
}
