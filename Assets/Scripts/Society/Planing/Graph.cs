using System;
using System.Collections.Generic;
using UnityEngine;

/*
 * This class represents a undirected graph with a visited mark on the nodes.
 * A graph is able to return a new univisited target in the current region the robot is in.
 * In addition it can return a path through the visited space to a new univisited target if the robot reached a dead end at the current position.
 * Nodes of the graph are variable in size. Each size is a circle of maximum radius without hitting a obstacle in the lastLaserReadings.
 * Corners are used to create intersections.
 */
class Graph {

    public const float EXTREMA_DISTANCE_CUTOFF = 2.5f * Planing.MIN_OBSTACLE_DISTANCE;//Take the robot's size into account!
    public const float MIN_NODE_DISTANCE = EXTREMA_DISTANCE_CUTOFF;
    public const float ROBOT_NODE_DISTANCE = MIN_NODE_DISTANCE;
    
    //TODO: DISPLAY NODES
    private List<GraphNode> nodes = new List<GraphNode>();//sorted by the x coordinate of pose + centerOffset.
    private int clientMapCount = 0;
    private int lastNode = -1;
    private Vector3 lastMatch = Vector3.zero;

    private class GraphNode {
        public RobotPose pose = null;
        public ulong timestamp;//The timestamp is used to find the closest match between the laserReadings and the clientMap.
        public Vector2 centerOffset;
        public float radius;//Radius is the euclidean distance to the nearest feature.
        public List<GraphNode> connectedNodes = new List<GraphNode>();

        public GraphNode(ulong timestamp, Vector2 centerOffset, float radius) {
            this.timestamp = timestamp;
            this.centerOffset = centerOffset;
            this.radius = radius;
        }
    }

    //Builds the graph with the provided laser readings.
    public void Feed(PlaningInputData lastLaserReadings) {
        lastLaserReadings.LastPose += lastMatch;
        GraphNode closestPoseNode = null;
        float closestDistance = float.MaxValue;
        foreach (GraphNode node in nodes) {
            var currentDistance = Geometry.EuclideanDistance(lastLaserReadings.LastPose, node.pose + node.centerOffset) - node.radius;
            if (closestDistance < currentDistance) {
                closestDistance = currentDistance;
                closestPoseNode = node;
            }
        }
        if(closestDistance > ROBOT_NODE_DISTANCE + closestPoseNode.radius) {
            //Find the closest obstacle:
            float closestReading = lastLaserReadings.ReadingsRB[0].x;
            for (int i = 1; i < lastLaserReadings.Readings.Length; i++) {
                if (lastLaserReadings.ReadingsRB[i].x < closestReading) closestReading = lastLaserReadings.ReadingsRB[i].x;
            }
            //Create circle around current pose. The radius is the distance to the closest feature.
            var node = new GraphNode(lastLaserReadings.Timestamp, lastLaserReadings.LastPose, closestReading); //TODO: Eventually add a constant.
            nodes.Add(node);
            closestPoseNode.connectedNodes.Add(node);
            node.connectedNodes.Add(closestPoseNode);
            closestPoseNode = node;
        }
        int previousReading = lastLaserReadings.Readings.Length - 1; 
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            if (Math.Abs(lastLaserReadings.ReadingsRB[previousReading].x - lastLaserReadings.ReadingsRB[i].x) > EXTREMA_DISTANCE_CUTOFF) {
                //i is an extremum compared to the previousReading.
                Vector3 centerOffset = (lastLaserReadings.Readings[i] - lastLaserReadings.Readings[previousReading]) / 2f;
                Vector2 center = new Vector2(lastLaserReadings.Readings[i].x + centerOffset.x + lastMatch.x, lastLaserReadings.Readings[i].z + centerOffset.z + lastMatch.y);
                center = Geometry.Rotate(center, lastLaserReadings.LastPose, lastMatch.z);
                //Find closest node and closest node with radius for the new node described by center:
                closestDistance = float.MaxValue;
                GraphNode closestRadiusNode = null;
                float closestRadiusDistance = float.MaxValue;
                foreach (GraphNode node in nodes) {
                    var currentDistance = Geometry.EuclideanDistance(center, node.pose + node.centerOffset);
                    if (currentDistance > MIN_NODE_DISTANCE) {
                        var currentRadiusDistance = currentDistance - (node.radius > centerOffset.magnitude ? node.radius : centerOffset.magnitude);
                        if (closestRadiusDistance > currentRadiusDistance) {
                            closestRadiusDistance = currentRadiusDistance;
                            closestRadiusNode = node;
                        }
                    } else if (closestDistance > currentDistance) {
                        closestDistance = currentDistance;
                    }
                }
                //Add the node if it is not an already existing node:
                if (closestDistance > MIN_NODE_DISTANCE) {
                    var node = new GraphNode(lastLaserReadings.Timestamp, center, centerOffset.magnitude);
                    nodes.Add(node);
                    node.connectedNodes.Add(closestPoseNode);
                    if (closestRadiusDistance < 0f) {
                        node.connectedNodes.Add(closestRadiusNode);
                        closestRadiusNode.connectedNodes.Add(node);
                    }
                }
            }
            previousReading = i;
        }
    }

    public void Feed(GlobalClientMap clientMap) {
        lock(clientMap.globalStateCollection) {
            while (clientMapCount < clientMap.globalStateCollection.Count) {

                clientMapCount++;
            }
        }
        throw new NotImplementedException();
    }

    //Marks the current pose / node as a dead end.
    public void ReachedDeadEnd(Vector3 currentPose) {
        throw new NotImplementedException();
    }

    //Returns a new target in the "unexplored" territory the robot is atm in.
    public Vector2 GetNewTarget() {
        throw new NotImplementedException();
    }

    //Returns a path through already visited territory to a new unexplored node in the graph.
    public Queue<Vector2> GetUnexploredNodePath() {
        throw new NotImplementedException();
    }
}
