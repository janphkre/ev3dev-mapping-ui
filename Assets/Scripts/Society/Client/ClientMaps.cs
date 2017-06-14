/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Iterated SLSJF: A Sparse Local Submap Joining Algorithm with Improved Consistency *
 * See: http://www.araa.asn.au/acra/acra2008/papers/pap102s1.pdf                     *
 * Paper by Shoudong Huang, Zhan Wang, Gamini Dissanayake and Udo Frese              *
 *                                                                                   *
 * Building upon:                                                                    *
 * SLSJF: Sparse local submap joining filter for building large-scale maps           *
 * See: http://services.eng.uts.edu.au/~sdhuang/SLSJF_IEEE_TRO_final_2008_May_27.pdf *
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
 * Every local map (should /) has the same feature count                                       *
 * as the SLAM algorithm will create a new map every time the feature count reaches a cut off. *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
public class LocalClientMap : MessageBase {

    public FeatureCollection points;
    public CovarianceMatrix covariance;
    public int featureCount = 0;

    public LocalClientMap(System.Random random, int size) {
        covariance = new CovarianceMatrix(random);
        points = new FeatureCollection(size);
    }

    public Vector4 this[int i] {
        get { return points.map[i]; }
        set { points.map[i] = value; }
    }
}


public class GlobalClientMap {

    public const float ESTIMATION_ERROR_CUTOFF = 1f;//TODO:Tune
    public const float OBSERVATION_NOISE_SIGMA = 1f;
    public const float CHANGE_OF_ESTIMATE_CUTOFF = 1f;
    public const int MAX_SMOOTHING_ITERATIONS = 10;

    private System.Random random = new System.Random();
    private List<SparseCovarianceMatrix> localInversedCovarianceCollection = new List<SparseCovarianceMatrix>();//(P^L)^-1
    private List<List<Feature>> localStateCollection = new List<List<Feature>>();
    private SparseColumn infoVector = new SparseColumn(3);//i(k)
    public SparseCovarianceMatrix infoMatrix = new SparseCovarianceMatrix();//I(k)
    public SparseTriangularMatrix choleskyFactorization = new SparseTriangularMatrix();//L(k)
    public List<IFeature> globalStateVector = new List<IFeature>();//X^G(k)
    private List<List<Feature>> globalStateCollection = new List<List<Feature>>();
    RobotPose lastPose = RobotPose.zero;

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
            for (int i = 0; i < localMap.featureCount; i++) {
                Feature feat = new Feature(localMap.points.map[i], pose, globalStateVector.Count);
                globalStateVector.Add(feat);
                collection.Add(feat);
            }
            pose.index = globalStateVector.Count;
            lastPose = pose;
            globalStateVector.Add(pose);
            localStateCollection.Add(collection);
            globalStateCollection.Add(collection);
            //TODO: add the first map into the EIF (info+cholesky)
            throw new NotImplementedException();
        } else {
            /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
             * 2.1)Data association between local map k+1 and the global map (SLSJF) *
             * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
            List<int> unmatchedLocalFeatures;
            List<int> matchedGlobalFeatures;
            Vector3 match = DataAssociaton(localMap, out unmatchedLocalFeatures, out matchedGlobalFeatures);
            /* * * * * * * * * * * * * * * * *
             * 2.2) Initialization using EIF *
             * * * * * * * * * * * * * * * * */
            if (unmatchedLocalFeatures.Count == 0) return;//The local map does not contain any new information so we can quit at this point.
            //TODO:(Is this true?)
            //Calculate the global positions of all unmatched features
            localMap.points.end = new Vector3(localMap.points.end.x + match.x, localMap.points.end.y + match.y, localMap.points.end.z + match.z);
            RobotPose pose = new RobotPose(localMap.points.end, lastPose, localMap.points.radius);
            List<Feature> localCollection = new List<Feature>();
            List<Feature> globalCollection = new List<Feature>();
            int j = 0,
                k = 0;
            for (int i = 0; i < localMap.featureCount; i++) {
                if (i == unmatchedLocalFeatures[j]) {
                    //Move first, rotate second: rotation happens around the moved end pose
                    localMap.points.map[unmatchedLocalFeatures[j]].x += match.x;
                    localMap.points.map[unmatchedLocalFeatures[j]].y += match.y;
                    localMap.points.map[unmatchedLocalFeatures[j]].z += match.x;
                    localMap.points.map[unmatchedLocalFeatures[j]].w += match.y;
                    Feature feat = new Feature(Geometry.Rotate(localMap.points.map[unmatchedLocalFeatures[j++]], localMap.points.end, match.z), pose, globalStateVector.Count);
                    globalStateVector.Add(feat);
                    localCollection.Add(feat);
                    globalCollection.Add(feat);
                } else {
                    globalCollection.Add((Feature)globalStateVector[matchedGlobalFeatures[k++]]);
                }
            }
            pose.index = globalStateVector.Count;

            globalStateVector.Add(pose);
            localStateCollection.Add(localCollection);
            globalStateCollection.Add(globalCollection);
            //Enlarge the info vector, info matrix and cholesky factorization by adding zeros:
            //infoVector is a dictionary, so no enlarging is needed.
            infoMatrix.Enlarge2(unmatchedLocalFeatures.Count);
            infoMatrix.Enlarge3();
            choleskyFactorization.Enlarge2(unmatchedLocalFeatures.Count);
            choleskyFactorization.Enlarge3();
            /* * * * * * * * * * * * *
             * 2.3) Update using EIF *
             * * * * * * * * * * * * */
            //2.3.1) Compute the information matrix and vector using EIF
            SparseCovarianceMatrix localMapInversedCovariance = !localMap.covariance;
            localInversedCovarianceCollection.Add(localMapInversedCovariance);
            computeInfoAddition(globalCollection, localMapInversedCovariance);
            lastPose = pose;//TODO:move to end eventually
            //2.3.2) Reorder the global map state vector when necessary
            minimalDegreeReorder();
            //2.3.3) to 2.4) Cholesky, recover global state estimate and least squares smoothing:
            recursiveConverging(MAX_SMOOTHING_ITERATIONS);
        }
    }

    private Vector3 DataAssociaton(LocalClientMap localMap, out List<int> unmatchedLocalFeatures, out List<int> matchedGlobalFeatures) {
        List<int> prematchedFeatures = new List<int>();
        float estimatedRadius = localMap.points.radius + estimationError;
        Vector3 start = Vector3.zero;
        //2.1.1) Determine the set of potentially overlapping local maps:
        foreach (List<Feature> collection in localStateCollection) {
            RobotPose pose = collection[0].ParentPose();
            if ((localMap.points.end - start).magnitude <= estimatedRadius + pose.radius) {
                //2.1.2) Find the set of potentially matched features:
                for (int i = 0; i < collection.Count; i++) {
                    if (Geometry.MaxDistance(localMap.points.end, collection[i].feature) <= estimatedRadius) {
                        //The first index is the matched map; The second index is the matched feature in the matched map.
                        prematchedFeatures.Add(collection[i].index);//Like this the matchedFeatures should be sorted at all times.
                    }
                }
            }
            start = pose.pose;
        }
        //2.1.3) Recover the covariance submatrix associated with X^G_(ke) and the potentially matched features:
        SparseColumn q = new SparseColumn(2),
                     p;
        SparseColumn columnVector = new SparseColumn(2);
        SparseCovarianceMatrix subMatrix = new SparseCovarianceMatrix();
        for (int i = 0; i < prematchedFeatures.Count; i++) {
            columnVector[prematchedFeatures[i]] = new Matrix(2);
            solveLowerLeftSparse(choleskyFactorization, out q, columnVector);
            columnVector.Remove(prematchedFeatures[i]);
            solveUpperRightSparse(choleskyFactorization, out p, q);
            subMatrix.Add(p);
        }
        //Add the last robot position: 
        columnVector = new SparseColumn(3);
        columnVector[lastPose.index] = new Matrix(3);
        //Solve the sparse linear equations:
        solveLowerLeftSparse(choleskyFactorization, out q, columnVector);
        solveUpperRightSparse(choleskyFactorization, out p, q);
        subMatrix.Add(p);
        //Remove the unneeded rows from the submatrix:
        subMatrix.Trim(prematchedFeatures, globalStateVector.Count);
        //2.1.4) Nearest Neighbor or Joint Compatibility Test method to find the match:
        //TODO: estimationError überarbeiten: Muss sich aus den local maps ergeben
        if (estimationError >= ESTIMATION_ERROR_CUTOFF) {
            return pairDrivenGlobalLocalization(localMap, subMatrix, out unmatchedLocalFeatures, out matchedGlobalFeatures, prematchedFeatures);
        } else {
            return jointCompatibilityTest(localMap, subMatrix, out unmatchedLocalFeatures, out matchedGlobalFeatures, prematchedFeatures);
        }
    }

    private void solveLowerLeftSparse(SparseTriangularMatrix matrix, out SparseColumn result, SparseColumn rightHandSide) {
        result = new SparseColumn(3);
        int size = matrix.ColumnCount();
        for (int i = 0;i < size; i++) {//Rows
            if(matrix[i, i] != null) {
                Matrix m = rightHandSide[i];
                for(int j = 0; j < i; j++) {//Columns
                    m -= matrix[j, i] * result[j];
                }
                if (m != null) result[i] = m * !matrix[i, i];
            }
        }
    }

    private void solveUpperRightSparse(SparseTriangularMatrix matrix, out SparseColumn result, SparseColumn rightHandSide) {
        result = new SparseColumn(3);
        int size = matrix.ColumnCount();
        for (int i = size - 1; i >= 0; i++) {//Rows
            if (matrix[i, i] != null) {
                Matrix m = rightHandSide[i];
                for (int j = size - 1; j < i; j++) {//Columns
                    //TODO:If we just switch rows and cols in the matrix here we do not have to translate it.(right?)Could this be made faster by accessing the dictionary-key-enumerator directly with skipping the zeros over the column?
                    m -= matrix[i, j] * result[j];
                }
                if (m != null) result[i] = m * !matrix[i, i];
            }
        }
    }

    private Vector3 jointCompatibilityTest(LocalClientMap localMap, SparseCovarianceMatrix subMatrix, out List<int> unmatchedLocalFeatures, out List<int> matchedGlobalFeatures, List<int> prematchedFeatures) {
        unmatchedLocalFeatures = new List<int>();
        matchedGlobalFeatures = new List<int>();
        throw new NotImplementedException();
    }

    //Sollte sich auch auf dem server nutzen lassen um die einzelnen globalen karten zu einer zusammenzuführen
    private Vector3 pairDrivenGlobalLocalization(LocalClientMap localMap, SparseCovarianceMatrix subMatrix, out List<int> unmatchedLocalFeatures, out List<int> matchedGlobalFeatures, List<int> prematchedFeatures) {
        unmatchedLocalFeatures = new List<int>();
        matchedGlobalFeatures = new List<int>();
        throw new NotImplementedException();
    }

    private SparseMatrix computeJacobianH(List<Feature> localMapFeatures) {
        //H_k+1 = relative positions of robot and features in respect to previous global robot position and rotation
        FeatureCollection relativePositionsH = new FeatureCollection(localMapFeatures.Count);
        RobotPose currentPose = localMapFeatures[0].ParentPose();
        RobotPose previousPose = currentPose.PreviousPose();
        for (int i = 0; i < localMapFeatures.Count; i++) {
            Vector4 vec = (localMapFeatures[i]).feature;
            relativePositionsH.map[i] = new Vector4(
                (float)((vec.x - previousPose.pose.x) * Math.Cos(previousPose.pose.z) + (vec.y - previousPose.pose.y) * Math.Sin(previousPose.pose.z)),
                (float)((vec.y - previousPose.pose.y) * Math.Cos(previousPose.pose.z) - (vec.x - previousPose.pose.x) * Math.Sin(previousPose.pose.z)),
                (float)((vec.z - previousPose.pose.x) * Math.Cos(previousPose.pose.z) + (vec.w - previousPose.pose.y) * Math.Sin(previousPose.pose.z)),
                (float)((vec.w - previousPose.pose.y) * Math.Cos(previousPose.pose.z) - (vec.z - previousPose.pose.x) * Math.Sin(previousPose.pose.z))
            );
        }
        relativePositionsH.end = new Vector3(
            (float)((currentPose.pose.x - previousPose.pose.x) * Math.Cos(previousPose.pose.z) + (currentPose.pose.y - previousPose.pose.y) * Math.Sin(previousPose.pose.z)),
            (float)((currentPose.pose.y - previousPose.pose.y) * Math.Cos(previousPose.pose.z) - (currentPose.pose.x - previousPose.pose.x) * Math.Sin(previousPose.pose.z)),
            currentPose.pose.z - previousPose.pose.z
        );

        SparseMatrix jacobianH = new SparseMatrix();
        //TODO:what are the rows and what are the cols???
        jacobianH.Enlarge3();
        jacobianH.Enlarge2(localMapFeatures.Count);
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
            n[0, 2] = relativePositionsH.map[i].y;//TODO: zu center umwandeln!
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

    private SparseColumn computeNoiseW(int localMapSize) {
        //noiseW = w_j = zero mean gaussian "observation noise"
        SparseColumn noiseW = new SparseColumn(2);
        Matrix m = new Matrix(3, 3);
        noiseW[0] = m;
        m[0, 0] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
        m[1, 1] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
        m[2, 2] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
        for (int i = 1; i <= localMapSize; i++) {
            m = new Matrix(2, 2);
            noiseW[i] = m;
            m[0, 0] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            m[1, 1] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            //TODO: Do [0,1] and [1,0] need noise too?
        }
        return noiseW;
    }

    private void computeInfoAddition(List<Feature> localMapFeatures, SparseCovarianceMatrix localMapInversedCovariance) {
        SparseMatrix jacobianH = computeJacobianH(localMapFeatures);
        SparseColumn noiseW = computeNoiseW(localMapFeatures.Count);
        
        SparseMatrix infoMatrixAddition = ~jacobianH * localMapInversedCovariance;
        infoVector.Addition(infoMatrixAddition * (noiseW + jacobianH * globalStateVector));
        infoMatrixAddition *= jacobianH;
        infoMatrix.Addition(infoMatrixAddition);
    }
    
    private void minimalDegreeReorder() {
        throw new NotImplementedException();//TODO: ACTUAL AMD REORDERING
        int[] result;
        //Sort the information matrix and the information vector accordingly:
        HashSet<int> set = new HashSet<int>();
        Dictionary<int, Matrix> dictVector = new Dictionary<int, Matrix>();
        Dictionary<int, List<Matrix>> dictMatrix = new Dictionary<int, List<Matrix>>();
        Matrix m;
        for (int i = 0; i < result.Length; i++) {
            if (!set.Contains(i)) {
                //The i-th item was not used before: store it in dictData
                set.Add(i);
                dictVector.Add(i, infoVector[i]);
                List<Matrix> list = new List<Matrix>();
                SparseColumn col = infoMatrix.GetColumn(i);
                for (int j = 0; j < result.Length; j++) list.Add(col[j]);
                dictMatrix.Add(i, list);
            }
            if (set.Contains(result[i])) {
                //The result item can be found in the dictionary (old sorting):
                infoVector[i] = dictVector[result[i]];
                for (int j = 0; j < result.Length; j++) {
                    if ((i > j && result[i] < result[j]) || (i < j && result[i] > result[j])) {
                        //The matrix field has to be flipped from the (old) matrix:
                        if (set.Contains(result[j])) m = dictMatrix[result[j]][result[i]];
                        else m = infoMatrix[result[j], result[i]];
                        if (m != null) infoMatrix[i, j] = ~m;
                    } else {
                        m = dictMatrix[result[i]][result[j]];
                        if (m != null) infoMatrix[i, j] = m;
                    }
                }
            } else {
                //The result item can not be found in the dictionary:
                infoVector[i] = infoVector[result[i]];
                for (int j = 0; j < result.Length; j++) {
                    if ((i > j && result[i] < result[j]) || (i < j && result[i] > result[j])) {
                        //The matrix field has to be flipped from the (old) matrix:
                        if (set.Contains(result[j])) m = dictMatrix[result[j]][result[i]];
                        else m = infoMatrix[result[j], result[i]];
                        if (m != null) infoMatrix[i, j] = ~m;
                    } else {
                        m = infoMatrix[result[i], result[j]];
                        if (m != null) infoMatrix[i, j] = m;
                    }
                }
            }
        }
    }

    private SparseTriangularMatrix computeCholesky() {
        throw new NotImplementedException();
    }

    private void recursiveConverging(int maxIterations) {
        //2.3.3) and 2.4.2) Compute the Cholesky Factorization of I(k+1)
        choleskyFactorization = computeCholesky();
        //2.3.4) and 2.4.3) Recover the global map state estimate X^G(k+1)
        SparseColumn y, globalStateVectorNew;
        solveLowerLeftSparse(choleskyFactorization, out y, infoVector);
        solveUpperRightSparse(choleskyFactorization, out globalStateVectorNew, y);
        float changeOfEstimate = 0f;
        for(int i=0;i<globalStateVector.Count;i++) {
            IFeature v = globalStateVector[i];
            if(v.IsFeature()) {
                Feature f = (Feature) v;
                f.feature.z = ;
                f.feature.w = ;
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
}
