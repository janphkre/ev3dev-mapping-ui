using Superbest_random;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

class ISLSJFBase {

    public const float OBSERVATION_NOISE_SIGMA = 1f;
    public const int VERTICES_COUNT = SLAMRobot.MAX_MAP_SIZE * 16;

    private System.Random random = new System.Random();

    public ISLSJFBase() { }

    //Points enumerates Vector2 or Vector3
    public static void DisplayPoints(IEnumerator points, Map3D map, float height) {
        map.Clear();
        Vector3[] vertices = new Vector3[VERTICES_COUNT];
        bool b = true;
        while(b) {
            int i = 0;
            while (i < VERTICES_COUNT) {
                b = points.MoveNext();
                if (!b) break;
                vertices[i].x = ((Vector2) points.Current).x;
                vertices[i].z = ((Vector2) points.Current).y;
                vertices[i++].y = height;
            }
            map.AssignVertices(vertices, VERTICES_COUNT);
            vertices = new Vector3[VERTICES_COUNT];
        }
    }

    public List<Feature> OffsetLocalMap(RobotPose pose, Vector3 localMapPose, IArray<Vector2> localFeatures, List<int> unmatchedLocalFeatures, List<int> matchedGlobalFeatures, List<IFeature> globalStateVector) {
        Vector3 localMatchOffset = pose.pose - localMapPose;
        //Calculate the global positions of all unmatched features
        List<Feature> globalCollection = new List<Feature>();
        int j = 0;
        var matchedGlobalFeaturesEnumerator = matchedGlobalFeatures.GetEnumerator();
        matchedGlobalFeaturesEnumerator.MoveNext();
        for (int i = 0; i < localFeatures.Count; i++) {
            if (i == unmatchedLocalFeatures[j]) {
                //Move first, rotate second: rotation happens around the moved end pose
                Vector2 v = new Vector2(localFeatures[unmatchedLocalFeatures[j]].x + localMatchOffset.x, localFeatures[unmatchedLocalFeatures[j]].y + localMatchOffset.y);
                Feature feature = new Feature(Geometry.Rotate(v, pose.pose, localMatchOffset.z), pose, globalStateVector.Count);
                j++;
                globalStateVector.Add(feature);
                globalCollection.Add(feature);
            } else if(i == matchedGlobalFeaturesEnumerator.Current) {
                globalCollection.Add((Feature) globalStateVector[matchedGlobalFeaturesEnumerator.Current]);
                matchedGlobalFeaturesEnumerator.MoveNext();
            }
        }
        matchedGlobalFeaturesEnumerator.Dispose();
        pose.index = globalStateVector.Count;
        globalStateVector.Add(pose);
        return globalCollection;
    }

    private SparseMatrix ComputeJacobianH(RobotPose previousPose, List<Feature> localMapFeatures) {
        //H_k+1 = relative positions of robot and features in respect to previous global robot position and rotation
        Vector2[] relativePositionsH = new Vector2[localMapFeatures.Count];
        RobotPose currentPose = localMapFeatures[0].ParentPose();
        for (int i = 0; i < localMapFeatures.Count; i++) {
            Vector2 vec = (localMapFeatures[i]).feature;
            relativePositionsH[i] = new Vector2(
                ((vec.x - previousPose.pose.x) * Mathf.Cos(previousPose.pose.z) + (vec.y - previousPose.pose.y) * Mathf.Sin(previousPose.pose.z)),
                ((vec.y - previousPose.pose.y) * Mathf.Cos(previousPose.pose.z) - (vec.x - previousPose.pose.x) * Mathf.Sin(previousPose.pose.z))
            );
        }
        Vector3 relativePoseH = new Vector3(
            ((currentPose.pose.x - previousPose.pose.x) * Mathf.Cos(previousPose.pose.z) + (currentPose.pose.y - previousPose.pose.y) * Mathf.Sin(previousPose.pose.z)),
            ((currentPose.pose.y - previousPose.pose.y) * Mathf.Cos(previousPose.pose.z) - (currentPose.pose.x - previousPose.pose.x) * Mathf.Sin(previousPose.pose.z)),
            currentPose.pose.z - previousPose.pose.z
        );

        SparseMatrix jacobianH = new SparseMatrix();
        //TODO:what are the rows and what are the cols???
        jacobianH.Enlarge(localMapFeatures.Count + 1);
        //jacobian first three rows:
        Matrix l = new Matrix(3, 3);
        l[0, 0] = -Mathf.Cos(previousPose.pose.z);
        l[0, 1] = -Mathf.Sin(previousPose.pose.z);
        l[0, 2] = relativePoseH.y;
        l[1, 0] = -l[0, 1];
        l[1, 1] = l[0, 0];
        l[1, 2] = -relativePoseH.x;
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
            n[0, 2] = relativePositionsH[i].y;
            n[1, 0] = m[0, 1];
            n[1, 1] = l[0, 0];
            n[1, 2] = -relativePositionsH[i].x;
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

    private SparseColumn ComputeNoiseW(List<Feature> localMapFeatures) {
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

    public void computeInfoAddition(RobotPose previousPose, List<Feature> features, SparseCovarianceMatrix inversedCovariance, SparseCovarianceMatrix infoMatrix, SparseColumn infoVector, List<IFeature> globalStateVector) {
        SparseMatrix jacobianH = ComputeJacobianH(previousPose, features);
        //SparseColumn noiseW = computeNoiseW(localMapFeatures);

        SparseMatrix infoMatrixAddition = ~jacobianH * inversedCovariance;
        infoVector.Addition(infoMatrixAddition * (/*noiseW +*/ jacobianH * globalStateVector));//TODO:is the noise needed? Or is it actually generated by the sensor?
        infoMatrixAddition *= jacobianH;
        infoMatrix.Addition(infoMatrixAddition);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * See http://www.cs.utexas.edu/~pingali/CS378/2011sp/lectures/chol4.pdf *
     * The Cholesky Factorization                                            *
     * by Keshav Pingali                                                     *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Matrix Computations, 4th Edition                                      *
     * by Gene H. Golub & Charles F. Van Loan                                *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    public SparseTriangularMatrix ComputeCholesky(SparseCovarianceMatrix inputMatrix, Func<int, int> indexSize) {
        SparseTriangularMatrix choleskyFactorization = new SparseTriangularMatrix();
        choleskyFactorization.Enlarge(inputMatrix.ColumnCount());
        IEnumerator<SparseColumn> choleskyEnum = choleskyFactorization.GetColumnEnumerator();
        choleskyEnum.Reset();
        int c = 0;
        foreach (SparseColumn col in inputMatrix.val) {
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
        foreach (SparseColumn col in inputMatrix.val) {
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
                choleskyC[i, i] = Mathf.Sqrt(choleskyC[i, i]);
                //No need to continue with the other elements of the column as we would be dividing by zero.
                if (choleskyC[i, i] == 0.0f) continue;
                //Other elements j of submatrix c in column i
                for (int m = i + 1; m < col[c].sizeY; m++) {
                    for (int l = 0; l < c; l++) {
                        Matrix choleskyL = choleskyFactorization[l, c];
                        if (choleskyL != null)
                            for (int j = 0; j < choleskyL.sizeX; j++) choleskyC[i, m] -= choleskyL[j, m] * choleskyL[j, i];
                    }
                    for (int j = 0; j < i; j++) choleskyC[i, m] -= choleskyC[j, m] * choleskyC[j, i];
                    choleskyC[i, m] /= choleskyC[i, i];
                }
                //All elements m of submatrix k in column i
                for (int k = c + 1; k < inputMatrix.ColumnCount(); k++) {
                    Matrix choleskyK = choleskyEnum.Current[k];
                    if (choleskyK == null) {
                        choleskyK = new Matrix(col[c].sizeX, indexSize(c));
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
            for (int i = c; i < inputMatrix.ColumnCount(); i++) if (choleskyEnum.Current[i].IsEmpty()) choleskyEnum.Current.Remove(i);
            c++;
        }
        choleskyEnum.Dispose();
        return choleskyFactorization;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * See http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.25.8417 *
     * An Approximate Minimum Degree Ordering Algorithm                    *
     * by Rue Camichel Toulouse, P.R. Amestoy, T.A. Davis, I.S. Duff,      *
     * Patrick Amestoy , Timothy A. Davis , Iain , S. Duff                 *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    public void MinimumDegreeReorder(SparseCovarianceMatrix inputMatrix, SparseColumn inputVector1, List<IFeature> inputVector2) {
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
        for (int i = 0; i < inputVector2.Count; i++) {
            v.Add(i);
            var aI = new HashSet<int>();
            var col = inputMatrix.GetColumn(i).val;
            foreach (int j in col.Keys) {
                //For convenience the two/three columns of each column for the coordinates of the robot poses and features are not stripped.
                if (j != i) aI.Add(j);
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
        while (k < inputVector2.Count) {
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
            int[] w = new int[inputVector2.Count];
            for (int i = 0; i < w.Length; i++) w[i] = -1;
            //TODO: make sure that lP only contains SUPERVARIABLES
            foreach (int i in lP) {
                foreach (int eI in e[i]) {
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
                    if (e[vI].Remove(i)) e[vI].Add(vP);
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
                int min = inputVector2.Count - k;
                //Second minimum:
                int lPICount = lP.Count;
                foreach (int superI in super[i]) {
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
                for (int j = lP.Count - 1; j > i; j--) {
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
        //Sort the information matrix, the information vector and the globalStateVector accordingly:
        HashSet<int> set = new HashSet<int>();
        Dictionary<int, Matrix> dictVector = new Dictionary<int, Matrix>();
        Dictionary<int, IFeature> dictState = new Dictionary<int, IFeature>();
        Dictionary<int, Dictionary<int, Matrix>> dictMatrix = new Dictionary<int, Dictionary<int, Matrix>>();
        Matrix m;
        var result = vBar.GetEnumerator();
        for (int i = 0; i < vBar.Count; i++) {
            result.MoveNext();
            if (!set.Contains(i)) {
                //The i-th item was not used before; Store it in dictionary:
                set.Add(i);
                dictVector.Add(i, inputVector1[i]);
                dictState.Add(i, inputVector2[i]);
                var list = new Dictionary<int, Matrix>();
                var col = inputMatrix.GetColumn(i);
                foreach(KeyValuePair<int, Matrix> pair in col.val) {
                    list.Add(pair.Key, pair.Value);
                }
                dictMatrix.Add(i, list);
            }
            if (set.Contains(result.Current)) {
                //The result item can be found in the dictionary (old sorting):
                inputVector1[i] = dictVector[result.Current];
                var state = dictState[result.Current];
                state.index = i;
                inputVector2[i] = state;
                int j = 0;
                foreach (int resJ in vBar) {
                    if ((i > j && result.Current < resJ) || (i < j && result.Current > resJ)) {
                        //The matrix field has to be flipped from the (old) matrix:
                        if (set.Contains(resJ)) m = dictMatrix[resJ][result.Current];
                        else m = inputMatrix[resJ, result.Current];
                        if (m != null) inputMatrix[i, j] = ~m;
                    } else {
                        m = dictMatrix[result.Current][resJ];
                        if (m != null) inputMatrix[i, j] = m;
                    }
                    j++;
                }
            } else {
                //The result item can not be found in the dictionary:
                set.Add(result.Current);
                inputVector1[i] = inputVector1[result.Current];
                var state = inputVector2[result.Current];
                state.index = i;
                inputVector2[i] = state;
                int j = 0;
                foreach (int resJ in vBar) {
                    if ((i > j && result.Current < resJ) || (i < j && result.Current > resJ)) {
                        //The matrix field has to be flipped from the (old) matrix:
                        if (set.Contains(resJ)) m = dictMatrix[resJ][result.Current];
                        else m = inputMatrix[resJ, result.Current];
                        if (m != null) inputMatrix[i, j] = ~m;
                    } else {
                        m = inputMatrix[result.Current, resJ];
                        if (m != null) inputMatrix[i, j] = m;
                    }
                    j++;
                }
            }
        }
        result.Dispose();
    }
}
}
