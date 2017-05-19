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

    public PointCollection points = new PointCollection();
    public CovarianceMatrix covariance = new CovarianceMatrix();
    public Vector3 startRobotPos;
    public int featureCount = 0;

    public LocalClientMap() { }

    public LocalClientMap(int size, Vector3 start) {
        points.map = new Vector4[size];
        startRobotPos = start;
    }

    public Vector4 this[int i] {
        get { return points.map[i]; }
        set { points.map[i] = value; }
    }
}

public class GlobalClientMap {

    public const float ESTIMATION_ERROR = 1;

    public LinkedList<PointCollection> maps = new LinkedList<PointCollection>();
    public Vector infoVector = new Vector();
    public Matrix infoMatrix = new Matrix();

    /* * * * * * * * * * * * * * * * * * * * * * * *
     * Algorithm 1 & 2 of Iterated SLSJF.          *
     * Only completed local maps must be provided. *
     * * * * * * * * * * * * * * * * * * * * * * * */
    public void ConsumeLocalMap(LocalClientMap localMap) {
        if (localMap.points.map.Length == 0) return;
        if (maps.Count == 0) {
            /* * * * * * * * * * * * * * * * * * * * * *
             * 1) Set local map 1 as the global map  *
             * * * * * * * * * * * * * * * * * * * * * */
            maps.AddLast(new PointCollection(localMap.points));
        } else {
            /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
             * 2.1)Data association between local map k+1 and the global map (SLSJF) *
             * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
            LinkedList<Vector2> matchedFeatures = new LinkedList<Vector2>();
            LinkedList<Vector2> unmatchedLocalFeatures = new LinkedList<Vector2>();
            float estimatedRadius = localMap.points.radius + ESTIMATION_ERROR;
            int i = 0;
            //2.1.1) Determine the set of potentially overlapping local maps:
            foreach (PointCollection map in maps) {
                if ((localMap.points.start - map.start).magnitude <= estimatedRadius + map.radius) {
                    //2.1.2) Find the set of potentially matched features:
                    for (i = 0; i < map.map.Length; i++) {
                        if ((localMap.points.start - map.map[i]).magnitude <= estimatedRadius) matchedFeatures.AddLast(map.map[i]);
                    }
                }
            }
            //2.1.3) Recover the covariance submatrix associated with X^G_(ke) and the potentially matched features:
            /*TODO!*/
            //2.1.4) Nearest Neighbor method to find the match:

            /* * * * * * * * * * * * * * * * *
             * 2.2) Initialization using EIF *
             * * * * * * * * * * * * * * * * */
            if (unmatchedLocalFeatures.Count == 0) return;//TODO: THIS IS CERTAINLY NOT TRUE: The local map does not contain any new information so we can quit at this point.
            PointCollection globalMap = new PointCollection();
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
}
