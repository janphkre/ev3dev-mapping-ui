using System;
using System.Collections.Generic;
using UnityEngine;

class GraphNode {

    public RobotPose pose = null;
    public Vector2 centerOffset;
    public float radius;//Radius is the euclidean distance to the nearest feature.
    public List<int> connectedNodes = new List<int>();
    public bool deadEnd = false;

    public GraphNode(Vector2 centerOffset, float radius) {
        this.centerOffset = centerOffset;
        this.radius = radius;
    }
}

/*
 * This class represents an undirected graph with a visitation mark on the nodes.
 * A graph is able to return a new univisited target in the current region the robot is in.
 * In addition it can return a path through the visited space to a new univisited target if the robot reached a dead end at the current position.
 * Nodes of the graph are variable in size. Each size is a circle of maximum radius without hitting a obstacle in the lastLaserReadings.
 * Corners are used to create intersections.
 */
class Graph {

    public const float EXTREMA_DISTANCE_CUTOFF = 2.5f * Planing.MIN_OBSTACLE_DISTANCE;//Take the robot's size into account!
    public const float MIN_NODE_DISTANCE = EXTREMA_DISTANCE_CUTOFF;
    public const float ROBOT_NODE_DISTANCE = MIN_NODE_DISTANCE;
    
    //Synchronized:
    private List<GraphNode> nodes = new List<GraphNode>();
    private List<int> unvisitedNodes = new List<int>();
    private Vector3 lastMatch = Vector3.zero;
    private Vector3 matchPose = Vector3.zero;
    //Not synchronized:
    private int lastMatchedCounter = 0;//Used & maintained in Feed(List<List<Feature>>)
    private int lastNode = -1;//Used in ReachedDeadEnd(Vector3) & GetNewTarget() & GetUnexploredNodePath(); Maintained in Feed(PlaningInputData)

    //Builds the graph with the provided laser readings.
    public void Feed(PlaningInputData lastLaserReadings) {
        Vector3 currentPose = lastLaserReadings.LastPose + lastMatch;
        //TODO: Does the pose has to be rotated as well???
        float closestDistance = float.MaxValue;
        if (lastNode >= 0) closestDistance = Geometry.EuclideanDistance((Vector2) currentPose, nodes[lastNode].pose + nodes[lastNode].centerOffset) - nodes[lastNode].radius;
        if (closestDistance > ROBOT_NODE_DISTANCE) {
            if (lastNode >= 0) {
                int closestJ = -1;
                foreach (int j in nodes[lastNode].connectedNodes) {
                    float currentDistance = Geometry.EuclideanDistance((Vector2) currentPose, nodes[j].pose + nodes[j].centerOffset) - nodes[j].radius;
                    if (currentDistance < closestDistance) {
                        closestDistance = currentDistance;
                        closestJ = j;
                    }
                }
                if (closestJ >= 0) {
                    if (closestDistance <= ROBOT_NODE_DISTANCE) {
                        lastNode = closestJ;
                        unvisitedNodes.Remove(closestJ);
                    }
                }
            }
            if (closestDistance > ROBOT_NODE_DISTANCE) {
                //Find the closest obstacle:
                float closestReading = lastLaserReadings.ReadingsRB[0].x;
                for (int i = 1; i < lastLaserReadings.Readings.Length; i++) {
                    if (lastLaserReadings.ReadingsRB[i].x < closestReading) closestReading = lastLaserReadings.ReadingsRB[i].x;
                }
                //Create circle around current pose. The radius is the distance to the closest feature.
                var node = new GraphNode(lastLaserReadings.LastPose, closestReading); //TODO: Eventually add a constant to the radius.                
                if (lastNode >= 0) node.connectedNodes.Add(lastNode);
                lock (nodes) {
                    node.centerOffset += (Vector2)lastMatch; //TODO: Does the pose has to be rotated as well???
                    if (lastNode >= 0) nodes[lastNode].connectedNodes.Add(nodes.Count);
                    nodes.Add(node);
                }
                lastNode = nodes.Count - 1;
            }
        }
        int previousReading = lastLaserReadings.Readings.Length - 1; 
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            if (Mathf.Abs(lastLaserReadings.ReadingsRB[previousReading].x - lastLaserReadings.ReadingsRB[i].x) > EXTREMA_DISTANCE_CUTOFF) {
                //i is an extremum compared to the previousReading.
                Vector3 centerOffset = (lastLaserReadings.Readings[i] - lastLaserReadings.Readings[previousReading]) / 2f;
                Vector2 center;
                lock (nodes) {
                    center = new Vector2(lastLaserReadings.Readings[i].x + centerOffset.x + lastMatch.x, lastLaserReadings.Readings[i].z + centerOffset.z + lastMatch.y);
                    center = Geometry.Rotate(center, matchPose, lastMatch.z);
                }
                //Find closest node and closest node with radius for the new node described by center:
                closestDistance = float.MaxValue;
                int closestRadiusNode = 0;
                float closestRadiusDistance = Geometry.EuclideanDistance(center, nodes[0].pose + nodes[0].centerOffset);
                for (int j = 1; j < nodes.Count; j++) {
                    var currentDistance = Geometry.EuclideanDistance(center, nodes[j].pose + nodes[j].centerOffset);
                    if (currentDistance > MIN_NODE_DISTANCE) {
                        var currentRadiusDistance = currentDistance - (nodes[j].radius > centerOffset.magnitude ? nodes[j].radius : centerOffset.magnitude);
                        if (closestRadiusDistance > currentRadiusDistance) {
                            closestRadiusDistance = currentRadiusDistance;
                            closestRadiusNode = j;
                        }
                    } else if (closestDistance > currentDistance) {
                        closestDistance = currentDistance;
                    }
                }
                //Add the node if it is not an already existing node:
                if (closestDistance > MIN_NODE_DISTANCE) {
                    var node = new GraphNode(new Vector2(lastLaserReadings.Readings[i].x + centerOffset.x, lastLaserReadings.Readings[i].z + centerOffset.z), centerOffset.magnitude);
                    node.connectedNodes.Add(lastNode);
                    if (closestRadiusDistance < 0f) node.connectedNodes.Add(closestRadiusNode);
                    unvisitedNodes.Add(nodes.Count);
                    lock (nodes) {
                        node.centerOffset += (Vector2) lastMatch;
                        node.centerOffset = Geometry.Rotate(node.centerOffset, matchPose, lastMatch.z);
                        if (closestRadiusDistance < 0f) nodes[closestRadiusNode].connectedNodes.Add(nodes.Count);
                        nodes.Add(node);
                    }
                }
            }
            previousReading = i;
        }
    }

    //Adapts the graph onto the global client map.
    //The graph is always matched except for new nodes that have not been processed by this function yet.
    public void Feed(List<List<Feature>> clientMap, Vector3 currentMatch, Vector3 currentPose) {
        Vector3 matchDelta = currentMatch - lastMatch;
        while (true) {
            lock(nodes) {
                if (lastMatchedCounter >= nodes.Count) {
                    lastMatch = currentMatch;
                    matchPose = currentPose;
                    break;
                }
            }
            var node = nodes[lastMatchedCounter];
            node.centerOffset += (Vector2) matchDelta;
            node.centerOffset = Geometry.Rotate(node.centerOffset, currentPose, matchDelta.z);
            node.pose = clientMap[0][0].ParentPose();
            float closestDistance = Geometry.EuclideanDistance(node.centerOffset, (Vector2) node.pose.pose);
            for(int i = 1; i < clientMap.Count; i++) {
                float currentDistance = Geometry.EuclideanDistance(node.centerOffset, (Vector2) clientMap[i][0].ParentPose().pose);
                if (currentDistance < closestDistance) {
                    node.pose = clientMap[i][0].ParentPose();
                    closestDistance = currentDistance;
                }
            }
            node.centerOffset -= (Vector2) node.pose.pose;
            lastMatchedCounter++;
        }
    }

    //Marks the current pose / node as a dead end.
    /*public void ReachedDeadEnd(Vector3 currentPose) {
        throw new NotImplementedException();//TODO: Do we have to do this?
    }*/

    //Returns a new target in the "unexplored" territory the robot is atm in.
    public bool GetNewTarget(out Vector2 result) {
        foreach (int j in nodes[lastNode].connectedNodes) {
            if(unvisitedNodes.Contains(j)) {
                lock(nodes) result =  nodes[j].pose + nodes[j].centerOffset - (Vector2) lastMatch;
                return true;
            }
        }
        result = Vector2.zero;
        return false;
    }

    //Returns a path through already visited territory to a new unexplored node in the graph.
    public LinkedList<Vector2> GetUnexploredNodePath(Vector3 currentPose) {
        currentPose += lastMatch;
        //Find the closest target:
        int target = unvisitedNodes[0];
        float closestDistance = Geometry.EuclideanDistance((Vector2)currentPose, nodes[target].pose + nodes[target].centerOffset) - nodes[target].radius;
        for (int i = 1; i < unvisitedNodes.Count; i++) {
            float currentDistance = Geometry.EuclideanDistance((Vector2)currentPose, nodes[unvisitedNodes[i]].pose + nodes[unvisitedNodes[i]].centerOffset) - nodes[unvisitedNodes[i]].radius;
            if (currentDistance < closestDistance) {
                closestDistance = currentDistance;
                target = i;
            }
        }
        
        var result = new LinkedList<Vector2>();
        throw new NotImplementedException();
    }
}
