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

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * The local map is created by the SLAM algorithm in SLAMRobot.cs.                             *
 * Every local map (should have /) has the same feature count                                  *
 * as the SLAM algorithm will create a new map every time the feature count reaches a cut off. *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
public class LocalClientMap : MessageBase {

    public FeatureCollection points = new FeatureCollection();
    public CovarianceMatrix covariance;
    public Vector2 startRobotPos;
    public int featureCount = 0;

    /*public LocalClientMap(System.Random random) {
        covariance = new CovarianceMatrix(random);
    }*/

    public LocalClientMap(System.Random random, int size, Vector3 start) {
        covariance = new CovarianceMatrix(random);
        points.map = new Vector4[size];
        startRobotPos = new Vector2(start.x, start.y);
    }

    public Vector4 this[int i] {
        get { return points.map[i]; }
        set { points.map[i] = value; }
    }
}

//k is the count of fused local maps.
public class GlobalClientMap {

    public const float ESTIMATION_ERROR = 1;

    //public LinkedList<FeatureCollection> maps = new LinkedList<FeatureCollection>();//TODO: überarbeiten: muss P_L nicht auch gespeichert werden? Aktuell wird end roboterpose nicht gespeichert.
    public Vector infoVector = new Vector();//i(k)
    public SparseCovarianceMatrix infoMatrix = new SparseCovarianceMatrix();//I(k)
    public SparseTriangularMatrix choleskyFactorization = new SparseTriangularMatrix();//L(k)
    public Vector globalStateVectorEstimate = new Vector();

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
        } else {
            /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
             * 2.1)Data association between local map k+1 and the global map (SLSJF) *
             * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
            List<int> matchedFeatures = new List<int>();
            float estimatedRadius = localMap.points.radius + ESTIMATION_ERROR;
            int i = 0,
                j = 0;
            Vector2 start = Vector2.zero;
            //2.1.1) Determine the set of potentially overlapping local maps:
            foreach (FeatureCollection map in maps) {
                if ((localMap.startRobotPos - start).magnitude <= estimatedRadius + map.radius) {
                    //2.1.2) Find the set of potentially matched features:
                    for (i = 0; i < map.map.Length; i++) {
                        if (Geometry.MaxDistance(localMap.startRobotPos, map.map[i]) <= estimatedRadius) {
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
            solveLowerLeftSparse(choleskyFactorization, q, columnVector);
            p = new SparseColumn(3);
            solveUpperRightSparse(choleskyFactorization, p, q);
            subMatrix.Add(p);
            subMatrix.Trim(matchedFeatures, infoMatrix.ColumnCount());
            //2.1.4) Nearest Neighbor or Joint Compatibility Test method to find the match:
            LinkedList<Vector2> unmatchedLocalFeatures = new LinkedList<Vector2>();
            //TODO!
            /* * * * * * * * * * * * * * * * *
             * 2.2) Initialization using EIF *
             * * * * * * * * * * * * * * * * */
            if (unmatchedLocalFeatures.Count == 0) return;//TODO: THIS IS CERTAINLY NOT TRUE: The local map does not contain any new information so we can quit at this point.(Reobservation alters the EIF)
            FeatureCollection globalMap = new FeatureCollection();
            globalMap.map = new Vector2[unmatchedLocalFeatures.Count];
            i = 0;
            foreach (Vector2 feature in unmatchedLocalFeatures) {
                globalMap.map[i] = feature + localMapOffset;
            }
            globalMap.start = localMap.points.start + localMapOffset;
            maps.AddLast(globalMap);
            infoVector.AddEmptyRow();
            infoMatrix.AddEmptyRow();
            infoMatrix.AddEmptyColumn();
            /* * * * * * * * * * * * *
             * 2.3) Update using EIF *
             * * * * * * * * * * * * */
            //2.3.1) Compute the information matrix and vector using EIF
            //2.3.2) Reorder the global map state vector when necessary
            //2.3.3) Compute the Cholesky Factorization of I(k+1)
            //2.3.4) Recover the global map state estimate X^G(k+1)
            //2.4) Least squares for smoothing if necessary
        }
    }

    private void solveLowerLeftSparse(SparseTriangularMatrix matrix, SparseColumn result, SparseColumn rightHandSide) {
        int size = matrix.ColumnCount();
        for (int i = 0;i < size; i++) {//Rows
            if(matrix[i, i] != null) {
                Matrix m = rightHandSide[i];
                for(int j = 0; j < i; j++) {//Columns
                    Matrix n = matrix[j, i] * result[j];
                    if (n != null) m = m - n;
                }
                if (m != null) result[i] = m * matrix[i, i];
            }
        }
    }

    private void solveUpperRightSparse(SparseTriangularMatrix matrix, SparseColumn result, SparseColumn rightHandSide) {
        int size = matrix.ColumnCount();
        for (int i = size - 1; i >= 0; i++) {//Rows
            if (matrix[i, i] != null) {
                Matrix m = rightHandSide[i];
                for (int j = size - 1; j < i; j++) {//Columns
                    Matrix n = matrix[i, j] * result[j];//If we just switch rows and cols in the matrix here we don't have to translate it.(right?)
                    if (n != null) m = m - n;
                }
                if (m != null) result[i] = m * matrix[i, i];
            }
        }
    }
}
