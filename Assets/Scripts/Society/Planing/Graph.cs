using System;
using System.Collections.Generic;
using ev3dev.Society;
using UnityEngine;
using UnityEngine.Networking;

namespace ev3devMapping.Society {

[Serializable]
public class GraphNode {

    public RobotPose pose = null;
    public Vector2 centerOffset;
    public float radius;//Radius is the euclidean distance to the nearest feature.
    [SerializeField] private IntList connectedNodes = new IntList();
    [SerializeField] private HashSet<int> disconnectedNodes = new HashSet<int>();
    
    public Vector2 Position {
        get { lock(this) return pose + centerOffset; }
    }

    public GraphNode(Vector2 centerOffset, float radius) {
        this.centerOffset = centerOffset;
        this.radius = radius;
    }

    public void Add(int i) {
        connectedNodes.Add(i);
    }

    public bool IsConnected(int i) {
        return connectedNodes.Contains(i);
    }

    public List<int> Connected {
        get { return connectedNodes; }
    }
    
    public void Remove(int i) {
        disconnectedNodes.Add(i);
    }

    public bool IsDisconnected(int i) {
        return disconnectedNodes.Contains(i);
    }
}

/*
  TODO:
  THE LOCKING BETWEEN THE TWO FEED FUNCTIONS IN THIS GRAPH CAN BE IMPROVED:
  WHEN A FEED OF A MAP COMPLETES WHILE A LASER READING IS BEING PROCESSED,
  ALL NODES ARE UPDATED TO THE NEW COORDINATE SYSTEM BU THE CURRENT POSE IS STILL USING THE OLD MATCH.
  IN ADDITION A NODE MIGHT BE UPDATED WHILE IT IS PROCESSED BY THE LASER READING FEEDING, SO A ASSOCIATION BETWEEN NODES MIGHT BE MODE WHERE THERE ARE NONE.
  ALTHOUGH THIS SHOULD ONLY BE CRITICAL WHEN THE MATCH IS VERY FAR OFF. (AKA THE GLOBAL LOCALISATION IS USED)
  LAST BUT NOT LEAST THE ROBOT POSES ARE NOT ATOMIC.
  */

/*
 * This class represents an undirected graph with a visitation mark on the nodes.
 * A graph is able to return a new univisited target in the current region the robot is in.
 * In addition it can return a path through the visited space to a new univisited target if the robot reached a dead end at the current position.
 * Nodes of the graph are variable in size. Each size is a circle of maximum radius without hitting a obstacle in the lastLaserReadings.
 * Corners are used to create intersections.
 */
public class Graph : MonoBehaviour {

    public const int EXTREMA_CHECK_RANGE = 45;
    public const float EXTREMA_DISTANCE_CUTOFF = 2.5f * AbstractTargetCommand.MIN_OBSTACLE_DISTANCE;//Take the robot's size into account!
    public const float MIN_NODE_DISTANCE = EXTREMA_DISTANCE_CUTOFF;
    public const float ROBOT_NODE_DISTANCE = 0f;
    public const float ACO_MIN_DIFF = 0.1f;
    public const float ACO_FORGETTING = 0.5f;
    public const int SEND_FREQUENCY = 20;
    public const float MIN_NODE_SIZE = AbstractTargetCommand.MIN_OBSTACLE_DISTANCE * 2f;

    public GameObject CirclePrefab;
    public GameObject EdgePrefab;
    
    //Synchronized:
    private GraphNodeList nodes;
    private IntList unvisitedNodes;
    private Vector3 lastMatch = Vector3.zero;
    private Vector3 matchPose = Vector3.zero;
    //Not synchronized:
    private int lastMatchedCounter = 0;//Used & maintained in Feed(List<List<Feature>>)
    private int lastNode = -1;//Used in ReachedDeadEnd(Vector3) & GetNewTarget() & GetUnexploredNodePath(); Maintained in Feed(PlaningInputData)
    private int sendCounter = 0;
    private GraphMessage message;
    private CircleMap2D map;

    public void Awake() {
        nodes = new GraphNodeList();
        unvisitedNodes = new IntList();
        message = new GraphMessage(nodes, unvisitedNodes);
        map = new CircleMap2D(CirclePrefab, EdgePrefab);
    }

    //Builds the graph with the provided laser readings.
    public void Feed(PlaningInputData lastLaserReadings) {
        int currentCount = nodes.Count;
        Vector3 currentPose = lastLaserReadings.LastPose + lastMatch;
        //TODO: Does the pose need to be rotated as well???
        //Check whether we are still close enough to a node:
        float closestDistance = float.MaxValue;
        if (lastNode >= 0) {
            closestDistance = Geometry.EuclideanDistance((Vector2) currentPose, nodes[lastNode].Position) - nodes[lastNode].radius;
            foreach (int j in nodes[lastNode].Connected) {
                float currentDistance = Geometry.EuclideanDistance((Vector2) currentPose, nodes[j].Position) - nodes[j].radius;
                if (currentDistance <= ROBOT_NODE_DISTANCE) {
                    unvisitedNodes.Remove(j);
                    if (currentDistance < closestDistance) {
                        closestDistance = currentDistance;
                        lastNode = j;
                    }
                }
            }
        }
        if (closestDistance > ROBOT_NODE_DISTANCE) {
            //We are not close enough to a node anymore:
            if (closestDistance > ROBOT_NODE_DISTANCE) {
                //Find the closest obstacle:
                float closestReading = lastLaserReadings.ReadingsRB[0].x;
                for (int i = 1; i < lastLaserReadings.ReadingsCount; i++) {
                    if (lastLaserReadings.ReadingsRB[i].x < closestReading) closestReading = lastLaserReadings.ReadingsRB[i].x;
                }
                //Create circle around current pose. The radius is the distance to the closest feature.
                var node = new GraphNode(lastLaserReadings.LastPose, closestReading); //TODO: Eventually add a constant to the radius.                
                if (lastNode >= 0) {
                    nodes[lastNode].Add(nodes.Count);
                    node.Add(lastNode);
                }
                lock (nodes) {
                    node.centerOffset += (Vector2)lastMatch;
                    node.centerOffset = Geometry.Rotate(node.centerOffset, matchPose, lastMatch.z);
                    nodes.Add(node);
                }
                lastNode = nodes.Count - 1;
            }
        }

        //Find holes in the laserReading:
        float closestReadingDistance = float.MaxValue;
        int previousReading = lastLaserReadings.ReadingsCount - 1; 
        for (int i = 0; i < lastLaserReadings.ReadingsCount; previousReading = i++) {
            if (closestReadingDistance > lastLaserReadings.ReadingsRB[i].x) closestReadingDistance = lastLaserReadings.ReadingsRB[i].x;
            if (Mathf.Abs(lastLaserReadings.ReadingsRB[previousReading].x - lastLaserReadings.ReadingsRB[i].x) > EXTREMA_DISTANCE_CUTOFF) {
                //i is an extremum compared to the previousReading.
                Vector3 centerOffset = (lastLaserReadings.Readings[previousReading] - lastLaserReadings.Readings[i]) / 2f;
                float radius = centerOffset.magnitude;
                Vector2 center;
                lock (nodes) {
                    center = new Vector2(lastLaserReadings.Readings[i].x + centerOffset.x + lastMatch.x, lastLaserReadings.Readings[i].z + centerOffset.z + lastMatch.y);
                    center = Geometry.Rotate(center, matchPose, lastMatch.z);
                }
                //Find closest node and closest node with radius for the new node described by center:
                int closestNode = 0;
                closestDistance = Geometry.EuclideanDistance(center, nodes[0].Position);
                int closestRadiusNode = 0;
                float closestRadiusDistance = closestDistance - (nodes[0].radius > centerOffset.magnitude ? nodes[0].radius : centerOffset.magnitude);
                for (int j = 1; j < nodes.Count; j++) {
                    var currentDistance = Geometry.EuclideanDistance(center, nodes[j].Position);
                    if (currentDistance > MIN_NODE_DISTANCE) {
                        var currentRadiusDistance = currentDistance - (nodes[j].radius > radius ? nodes[j].radius : radius);
                        if (closestRadiusDistance > currentRadiusDistance) {
                            closestRadiusDistance = currentRadiusDistance;
                            closestRadiusNode = j;
                        }
                    } else if (closestDistance > currentDistance) {
                        closestDistance = currentDistance;
                        closestNode = j;
                    }
                }
                //Add the node if it is not an already existing node:
                if (closestDistance <= MIN_NODE_DISTANCE || radius < MIN_NODE_SIZE) {
                    //Connect the two nodes:
                    if(closestNode != lastNode) {
                        connectNodes(closestNode, lastNode);
                    }
                    //Grow the closestNode:
                    nodes[closestRadiusNode].radius += radius;
                } else {
                    int j = 1;
                    //Make sure that we are not looking at bars or similiar things:
                    if (lastLaserReadings.ReadingsRB[previousReading].x < lastLaserReadings.ReadingsRB[i].x) {
                        //previous is closer than i.
                        for (; j <= EXTREMA_CHECK_RANGE; j++) {
                            if (Mathf.Abs(lastLaserReadings.ReadingsRB[previousReading].x - lastLaserReadings.ReadingsRB[(i + j) % lastLaserReadings.ReadingsCount].x) < EXTREMA_DISTANCE_CUTOFF) break;
                        }
                    } else {
                        //i is closer than previous.
                        for (; j <= EXTREMA_CHECK_RANGE; j++) {
                            if (Mathf.Abs(lastLaserReadings.ReadingsRB[i].x - lastLaserReadings.ReadingsRB[Geometry.Modulo((i - j), lastLaserReadings.ReadingsCount)].x) < EXTREMA_DISTANCE_CUTOFF) break;
                        }
                    }
                    if (j <= EXTREMA_CHECK_RANGE || closestRadiusDistance < 0f) continue;
                    //Add node:
                    var node = new GraphNode(center, radius);
                    nodes[lastNode].Add(nodes.Count);
                    node.Add(lastNode);
                    unvisitedNodes.Add(nodes.Count);
                    lock (nodes) {
                        node.centerOffset += (Vector2) lastMatch;
                        node.centerOffset = Geometry.Rotate(node.centerOffset, matchPose, lastMatch.z);
                        nodes.Add(node);
                    }
                    //Check if nodes are overlapping:
                    if(closestNode != lastNode && closestDistance < radius + nodes[closestNode].radius) {
                        connectNodes(closestNode, nodes.Count - 1);
                    }
                }
            }
        }
        for(int i = 0; i < currentCount; i++) {
            //Connect all nodes that are closer than closestDistance.
            if(i == lastNode) continue;
            if(Geometry.EuclideanDistance(nodes[i].Position, (Vector2)currentPose) < closestReadingDistance) connectNodes(lastNode, i);
        }
        //Display nodes:
        DisplayNodes();
    }

    //Adapts the graph onto the global client map.
    //The graph is always matched except for new nodes that have not been processed by this function yet.
    public void Feed(List<List<Feature>> clientMap, Vector3 currentMatch, Vector3 currentPose) {
        Debug.LogError("Feeding ClientMap to Graph!");
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
            lock(node) {
                node.centerOffset += (Vector2)matchDelta;
                node.centerOffset = Geometry.Rotate(node.centerOffset, currentPose, matchDelta.z);
                node.pose = clientMap[0][0].ParentPose();
                float closestDistance = Geometry.EuclideanDistance(node.centerOffset, (Vector2)node.pose.pose);
                for (int i = 1; i < clientMap.Count; i++) {
                    float currentDistance = Geometry.EuclideanDistance(node.centerOffset, (Vector2)clientMap[i][0].ParentPose().pose);
                    if (currentDistance < closestDistance) {
                        node.pose = clientMap[i][0].ParentPose();
                        closestDistance = currentDistance;
                    }
                }
                node.centerOffset -= (Vector2)node.pose.pose;
            }
            lastMatchedCounter++;
        }
        //Send this graph to the server:
        sendCounter++;
        sendCounter %= SEND_FREQUENCY;
        if (sendCounter == 0) NetworkManager.singleton.client.SendUnreliable((short)MessageType.ClientGraph, message);
    }

    public void DisplayNodes() {
        map.ProcessNodes(nodes, unvisitedNodes);
    }

    //Returns a new target in the "unexplored" territory the robot is atm in.
    public bool GetNewTarget(out Vector2 result) {
        if (lastNode >= 0) {
            foreach (int j in nodes[lastNode].Connected) {
                if (unvisitedNodes.Contains(j)) {
                    lock (nodes) result = nodes[j].Position - (Vector2)lastMatch;
                    return true;
                }
            }
        }
        result = Vector2.zero;
        return false;
    }

    public int UnvisitedNodeCount() {
        return unvisitedNodes.Count;
    }

    public bool HasUnvisitedNodes() {
        return unvisitedNodes.Count > 0;
    }

    //Returns a path to (0, 0) (assuming that nodes[0] is at (0, 0))
    public LinkedList<Vector2> GetStartPath(Vector3 currentPose) {
        return GetPath(currentPose, 0);
    }

    //Returns a path through already visited territory to a new unexplored node in the graph.
    public LinkedList<Vector2> GetUnexploredNodePath(Vector3 currentPose) {
        //Find the closest target:
        int target = unvisitedNodes[unvisitedNodes.Count - 1];
        float closestDistance = Geometry.EuclideanDistance((Vector2)currentPose, nodes[target].Position) - nodes[target].radius;
        //TODO:Unfortunately this for-loop is in the lock. Make this smarter.
        for (int i = 1; i < unvisitedNodes.Count; i++) {
            float currentDistance = Geometry.EuclideanDistance((Vector2)currentPose, nodes[unvisitedNodes[i]].Position) - nodes[unvisitedNodes[i]].radius;
            if (currentDistance < closestDistance) {
                closestDistance = currentDistance;
                target = i;
            }
        }
        return GetPath(currentPose, target);
    }

    private LinkedList<Vector2> GetPath(Vector3 currentPose, int target) {
        currentPose += lastMatch;
        var aco = new AntColonyOptimization(nodes, lastNode, target);
        float m = 0.0f,
              cost;
        do {
            cost = aco.Iteration();
            m = m * ACO_FORGETTING + cost * (1 - ACO_FORGETTING);
        } while (Mathf.Abs(m - cost) > ACO_MIN_DIFF);
        var result = new LinkedList<Vector2>();
        var path = aco.GetPath();
        lock (nodes) {
                foreach (int p in path) {
                    result.AddLast(nodes[p].Position);
                }
        }
        return result;
    }

    private void connectNodes(int i, int j) {
        if (!nodes[i].IsConnected(j) && !nodes[i].IsDisconnected(j)) {
            //no locking needed
            //lock(nodes) {
            nodes[i].Add(j);
            nodes[j].Add(i);
            //}
        }
    }

    public void DisconnectNode(Vector2 target) {
        int targetIndex = -1;
        float targetDistance = float.MaxValue;
        foreach(int i in nodes[lastNode].Connected) {
            float currentDistance = Geometry.EuclideanDistance(nodes[i].Position, target);
            if(currentDistance < targetDistance) {
                targetDistance = currentDistance;
                targetIndex = i;
            }
        }
        nodes[lastNode].Remove(targetIndex);
        nodes[targetIndex].Remove(lastNode);
        if(lastNode < targetIndex) map.RemoveEdge(lastNode, targetIndex);
        else map.RemoveEdge(targetIndex, lastNode);
    }
}
}