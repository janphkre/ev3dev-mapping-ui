using System;
using System.Collections.Generic;
using UnityEngine;

class Graph {

    public const float EXTREMA_DISTANCE_CUTOFF = 1.0f;//Take the robot's size into account!

    private class GraphNode {
        RobotPose pose;
        float radius;//Radius is the euclidean distance to the nearest feature.
    }

    //TODO:build a graph. This graph is feed by the GlobalClientMap(, LocalClientMap) and ServerMap.
    private Dictionary<int, GraphNode> nodes = new Dictionary<int, GraphNode>();
    private int clientMapCount = 0;

    public void Feed(GlobalClientMap clientMap) {
        while(clientMapCount < clientMap.globalStateCollection.Count) {

            clientMapCount++;
        }
    }

    public void Feed(PlaningInputData lastLaserReadings) {
        Vector3 previousReading = lastLaserReadings.Readings[lastLaserReadings.Readings.Length - 1];
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            float distance = Geometry.EuclideanDistance(previousReading, lastLaserReadings.Readings[i]);
            if (distance > EXTREMA_DISTANCE_CUTOFF) {
                //i is an extremum
                //Is the sudden change closer or farer from the current pose?
                float lastDistance = Geometry.EuclideanDistance(previousReading, lastLaserReadings.LastPos.position);
                distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], lastLaserReadings.LastPos.position);
                if (Geometry.EuclideanDistance(previousReading, (Vector2) lastLaserReadings.LastPose) > Geometry.EuclideanDistance(lastLaserReadings.Readings[i], lastLaserReadings.LastPos.position)) {
                    //The current item is closer than the previous reading.

                } else {

                }
            }
        }
        throw new NotImplementedException();
    }
}
