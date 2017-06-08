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

    private System.Random random = new System.Random();
    public SparseColumn infoVector = new SparseColumn(3);//i(k)
    public SparseCovarianceMatrix infoMatrix = new SparseCovarianceMatrix();//I(k)
    public SparseTriangularMatrix choleskyFactorization = new SparseTriangularMatrix();//L(k)
    public List<FeatureCollection> globalStateVector = new List<FeatureCollection>();

    /* * * * * * * * * * * * * * * * * * * * * * * *
     * Algorithm 1 & 2 of Iterated SLSJF.          *
     * Only completed local maps must be provided. *
     * * * * * * * * * * * * * * * * * * * * * * * */
    public void ConsumeLocalMap(LocalClientMap localMap) {
        if (localMap.points.map.Length == 0) return;
        if (maps.Count == 0) {
            /* * * * * * * * * * * * * * * * * * * * *
             * 1) Set local map 1 as the global map  *
             * * * * * * * * * * * * * * * * * * * * */
            maps.AddLast(new FeatureCollection(localMap.points));
            //TODO: add the first map into the EIF
            throw new NotImplementedException();
        } else {
            /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
             * 2.1)Data association between local map k+1 and the global map (SLSJF) *
             * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
            List<int> matchedFeatures = new List<int>();
            float estimatedRadius = localMap.points.radius + ESTIMATION_ERROR;
            int i = 0,
                j = 0;
            Vector3 start = Vector3.zero;
            //2.1.1) Determine the set of potentially overlapping local maps:
            foreach (FeatureCollection map in globalStateVector) {
                if ((localMap.points.end - start).magnitude <= estimatedRadius + map.radius) {
                    //2.1.2) Find the set of potentially matched features:
                    for (i = 0; i < map.map.Length; i++) {
                        if (Geometry.MaxDistance(localMap.points.end, map.map[i]) <= estimatedRadius) {
                            //The first index is the matched map; The second index is the matched feature in the matched map.
                            matchedFeatures.Add(j+i);//Like this the matchedFeatures should be sorted at all times.
                        }
                    }
                }
                start = map.end;
                j += map.map.Length + 1;//Each map has an additional 
            }
            //2.1.3) Recover the covariance submatrix associated with X^G_(ke) and the potentially matched features:
            SparseColumn q = new SparseColumn(2),
                         p;
            SparseColumn columnVector = new SparseColumn(2);
            SparseCovarianceMatrix subMatrix = new SparseCovarianceMatrix();
            for(i = 0; i < matchedFeatures.Count; i++) {
                columnVector[matchedFeatures[i]] = new Matrix(2);
                solveLowerLeftSparse(choleskyFactorization, q, columnVector);
                columnVector.Remove(matchedFeatures[i]);
                p = new SparseColumn(2);
                solveUpperRightSparse(choleskyFactorization, p, q);
                q.Clear();
                subMatrix.Add(p);
            }
            //Add the last robot position: 
            columnVector = new SparseColumn(3);
            columnVector[infoMatrix.ColumnCount()-1] = new Matrix(3);
            //Solve the sparse linear equations:
            solveLowerLeftSparse(choleskyFactorization, q, columnVector);
            p = new SparseColumn(3);
            solveUpperRightSparse(choleskyFactorization, p, q);
            subMatrix.Add(p);
            //Remove the unneeded rows from the submatrix:
            subMatrix.Trim(matchedFeatures, infoMatrix.ColumnCount());
            //2.1.4) Nearest Neighbor or Joint Compatibility Test method to find the match:
            List<int> unmatchedLocalFeatures = new List<int>();
            List<int> matchedLocalFeatures = new List<int>();
            Vector3 match;
            //TODO: ESTIMATION_ERROR überarbeiten: Muss sich aus den local maps ergeben
            if(ESTIMATION_ERROR >= ESTIMATION_ERROR_CUTOFF) {
                match = pairDrivenGlobalLocalization(localMap, subMatrix, out unmatchedLocalFeatures, out matchedLocalFeatures, out matchedFeatures);
            } else {
                match = jointCompatibilityTest(localMap, subMatrix, out unmatchedLocalFeatures, out matchedLocalFeatures, out matchedFeatures);
            }
            /* * * * * * * * * * * * * * * * *
             * 2.2) Initialization using EIF *
             * * * * * * * * * * * * * * * * */
            if (unmatchedLocalFeatures.Count == 0) return;//The local map does not contain any new information so we can quit at this point.
            //TODO:(Is this true?)
            //Calculate the global positions of all unmatched features
            FeatureCollection stateAddition = new FeatureCollection(unmatchedLocalFeatures.Count);
            localMap.points.end = stateAddition.end = new Vector2(localMap.points.end.x + match.x, localMap.points.end.y + match.y);
            for(i = 0; i < unmatchedLocalFeatures.Count; i++) {
                //Move first, rotatation second: rotation happens around the moved end Point
                localMap.points.map[unmatchedLocalFeatures[i]].x += match.x;
                localMap.points.map[unmatchedLocalFeatures[i]].y += match.y;
                localMap.points.map[unmatchedLocalFeatures[i]].z += match.x;
                localMap.points.map[unmatchedLocalFeatures[i]].w += match.y;
                stateAddition.map[i] = Geometry.Rotate(localMap.points.map[unmatchedLocalFeatures[i]], localMap.points.end, match.z);
            }
            globalStateVector.Add(stateAddition);
            //Enlarge the info vector, info matrix and cholesky factorization by adding zeros:
            //infoVector.Enlarge2(unmatchedLocalFeatures.Count);
            //infoVector.Enlarge3();
            int previousCount = infoMatrix.ColumnCount();
            infoMatrix.Enlarge2(unmatchedLocalFeatures.Count);
            infoMatrix.Enlarge3();
            choleskyFactorization.Enlarge2(unmatchedLocalFeatures.Count);
            choleskyFactorization.Enlarge3();
            /* * * * * * * * * * * * *
             * 2.3) Update using EIF *
             * * * * * * * * * * * * */
            //2.3.1) Compute the information matrix and vector using EIF
            Vector3 previousEnd = globalStateVector[globalStateVector.Count - 2].end;
            //H_k+1 = relative positions of robot and features in respect to previous global robot position and rotation
            FeatureCollection relativePositionsH = new FeatureCollection(localMap.featureCount);
            int k = 0;
            j = 0;
            for (i = 0; i < localMap.featureCount; i++) {
                if (i == unmatchedLocalFeatures[k]) {
                    //The i-th feature in the local map was not matched to an existing feature. We will use the global position calculated in the stateAddition.
                    relativePositionsH.map[i] = new Vector4(
                        (float) ((stateAddition.map[k].x - previousEnd.x) * Math.Cos(previousEnd.z) + (stateAddition.map[k].y - previousEnd.y) * Math.Sin(previousEnd.z)),
                        (float) ((stateAddition.map[k].y - previousEnd.y) * Math.Cos(previousEnd.z) - (stateAddition.map[k].x - previousEnd.x) * Math.Sin(previousEnd.z)),
                        (float) ((stateAddition.map[k].z - previousEnd.x) * Math.Cos(previousEnd.z) + (stateAddition.map[k].w - previousEnd.y) * Math.Sin(previousEnd.z)),
                        (float) ((stateAddition.map[k].w - previousEnd.y) * Math.Cos(previousEnd.z) - (stateAddition.map[k].z - previousEnd.x) * Math.Sin(previousEnd.z))
                    );
                    k++;
                } else {
                    //The i-th feature in the local map was matched to an existing feature. We will use the global position in the global state vector.
                    Vector4 feature = Geometry.GetFeature(globalStateVector, matchedFeatures[j++]);
                    relativePositionsH.map[i] = new Vector4(
                        (float) ((feature.x - previousEnd.x) * Math.Cos(previousEnd.z) + (feature.y - previousEnd.y) * Math.Sin(previousEnd.z)),
                        (float) ((feature.y - previousEnd.y) * Math.Cos(previousEnd.z) - (feature.x - previousEnd.x) * Math.Sin(previousEnd.z)),
                        (float) ((feature.z - previousEnd.x) * Math.Cos(previousEnd.z) + (feature.w - previousEnd.y) * Math.Sin(previousEnd.z)),
                        (float) ((feature.w - previousEnd.y) * Math.Cos(previousEnd.z) - (feature.z - previousEnd.x) * Math.Sin(previousEnd.z))
                    );
                }
            }
            relativePositionsH.end = new Vector3(
                (float) ((stateAddition.end.x - previousEnd.x) * Math.Cos(previousEnd.z) + (stateAddition.end.y - previousEnd.y) * Math.Sin(previousEnd.z)),
                (float) ((stateAddition.end.y - previousEnd.y) * Math.Cos(previousEnd.z) - (stateAddition.end.x - previousEnd.x) * Math.Sin(previousEnd.z)),
                stateAddition.end.z - previousEnd.z
            );

            SparseMatrix jacobianH = new SparseMatrix();
            //TODO:rows/cols
            jacobianH.Enlarge3();
            jacobianH.Enlarge2(localMap.featureCount);
            //jacobian first three rows:
            Matrix l = new Matrix(3, 3);
            l[0, 0] = (float) -Math.Cos(previousEnd.z);
            l[0, 1] = (float) -Math.Sin(previousEnd.z);
            l[0, 2] = (float) relativePositionsH.end.y;
            l[1, 0] = (float) -l[0, 1];
            l[1, 1] = (float) l[0, 0];
            l[1, 2] = (float) -relativePositionsH.end.x;
            l[2, 2] = -1f;
            jacobianH[0, previousCount - 1] = l;//X^G_ke
            
            Matrix m = new Matrix(3, 3);
            m[0, 0] = -l[0, 0];
            m[0, 1] = -l[0, 1];
            m[1, 0] = -l[1, 0];
            m[1, 1] = -l[1, 1];
            m[2, 2] = 1f;
            jacobianH[0, infoMatrix.ColumnCount()-1] = m;//X^G_k+1e
            //jacobian all other rows:
            k = 0;
            j = 0;
            for (i = 0; i < localMap.featureCount; i++) {
                Matrix n = new Matrix(2, 3);
                n[0, 0] = l[0, 0];
                n[0, 1] = l[0, 1];
                n[0, 2] = relativePositionsH.map[i].y;//TODO: zu center umwandeln!
                n[1, 0] = m[0, 1];
                n[1, 1] = l[0, 0];
                n[1, 2] = -relativePositionsH.map[i].x;
                jacobianH[i+1, previousCount - 1] = n;

                Matrix o = new Matrix(2, 2);
                o[0, 0] = m[0, 0];
                o[0, 1] = m[0, 1];
                o[0, 0] = l[0, 1];
                o[1, 1] = o[0, 0];
                if (i == unmatchedLocalFeatures[k]) {
                    jacobianH[previousCount + k, i + 1] = n;//According to mathlab code x and y are inverted : jacobianH[i + 1, previousCount + k]
                    k++;
                } else {
                    jacobianH[matchedFeatures[j], i + 1] = n;
                    j++;
                }
            }

            SparseColumn noiseW = new SparseColumn(2);//TODO: noiseW = w_j = zero mean gaussian "observation noise"
            noiseW[0] = new Matrix(3);
            noiseW[0][0, 0] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            noiseW[0][1, 1] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            noiseW[0][2, 2] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
            for (i = 1; i <= localMap.featureCount; i++) {
                noiseW[i] = new Matrix(2);
                noiseW[i][0, 0] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
                noiseW[i][1, 1] = RandomExtensions.NextGaussian(random, 0, OBSERVATION_NOISE_SIGMA);
                //TODO: Do [0,1] and [1,0] need noise too?
            }

            SparseMatrix infoMatrixAddition = ~jacobianH * !localMap.covariance;
            SparseColumn infoVectorAddition = infoMatrixAddition * (noiseW + jacobianH * globalStateVector);
            infoMatrixAddition *= jacobianH;

            infoMatrix.Addition(infoMatrixAddition);
            infoVector.Addition(infoVectorAddition);
            //2.3.2) Reorder the global map state vector when necessary

            //2.3.3) Compute the Cholesky Factorization of I(k+1)
            //2.3.4) Recover the global map state estimate X^G(k+1)
            //2.4) Least squares for smoothing if necessary
            throw new NotImplementedException();
        }
    }

    private void solveLowerLeftSparse(SparseTriangularMatrix matrix, SparseColumn result, SparseColumn rightHandSide) {
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

    private void solveUpperRightSparse(SparseTriangularMatrix matrix, SparseColumn result, SparseColumn rightHandSide) {
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

    private Vector3 jointCompatibilityTest(LocalClientMap localMap, SparseCovarianceMatrix subMatrix, out List<int> unmatchedLocalFeatures, out List<int> matchedLocalFeatures, out List<int> matchedFeatures) {
        throw new NotImplementedException();
    }

    //Sollte sich auch auf dem server nutzen lassen um die einzelnen globalen karten zu einer zusammenzuführen
    private Vector3 pairDrivenGlobalLocalization(LocalClientMap localMap, SparseCovarianceMatrix subMatrix, out List<int> unmatchedLocalFeatures, out List<int> matchedLocalFeatures, out List<int> matchedFeatures) {
        throw new NotImplementedException();
    }

}
