using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

public class CircleMap2D : Object {
    
    private class Circle {
        public GameObject GameObj { get; private set; }
        public List<int[]> Connected {get; private set; }
        
        public Circle(GameObject gameObj) {
                GameObj = gameObj;
                Connected = new List<int[]>();
        }
    }
    
    public const float MAP_HEIGHT = 0.0f;
    public const float ITEM_HEIGHT = 0.01f;
    private GameObject prefabCircle;
    private GameObject prefabEdge;
    private Transform container;
    private List<Circle> circles = new List<Circle>();
    private List<LineRenderer> edges = new List<LineRenderer>();
    
    public CircleMap2D(GameObject circle, GameObject edge) {
        prefabCircle = circle;
        prefabEdge = edge;
        container = new GameObject("CircleMap2D").transform;
        container.parent = SceneManager.DynamicObjects;
    }

    public void ProcessNodes(List<GraphNode> nodes, int count, List<int> unvisitedNodes) {
        if(nodes.Count == 0) return;
        int i = 0,
            j = 0,
            nextUnvisited = -1;
        if(unvisitedNodes.Count > 0) {
            nextUnvisited = unvisitedNodes[0];
        }
        foreach (Circle circle in circles) {
            Vector2 position = nodes[i].Position;
            circle.GameObj.transform.position = new Vector3(position.x, MAP_HEIGHT, position.y);
            var material = circle.GameObj.GetComponent<MeshRenderer>().material;
            if (i == nextUnvisited) {
                if(++j < unvisitedNodes.Count) {
                    nextUnvisited = unvisitedNodes[j];
                } else {
                    nextUnvisited = -1;
                }
            } else {
                material.color = Color.white;
            }
            for (int k = 0; k < circle.Connected.Count; k++) {
                //Existing edges:
                //circle.Connected[i] = nodes[i].Connected[k];
                edges[circle.Connected[k][0]].SetPosition(circle.Connected[k][1], circle.GameObj.transform.position);
                
            }
            for (int k = circle.Connected.Count; k < nodes[i].Connected.Count; k++) {
                //New edges:
                if (nodes[i].Connected[k] > i) {
                    int[] c = new int[2];
                    c[0] = nodes[i].Connected[k];
                    c[1] = 1;
                    circle.Connected.Add(c);
                } else {
                    int[] c = new int[2];
                    c[0] = edges.Count;
                    c[1] = 0;
                    circle.Connected.Add(c);
                    circles[nodes[i].Connected[k]].Connected[circles[nodes[i].Connected[k]].Connected.FindIndex((int[] l) => { return l[0] == i; })][0] = edges.Count;
                    edges.Add(createEdge(nodes[nodes[i].Connected[k]].Position, nodes[i].Position, "E" + nodes[i].Connected[k] + "," + i));
                }
            }
            i++;
        }
        while(i < count) {
            Circle circle = new Circle(Instantiate(prefabCircle, container));
            circle.GameObj.name = "C"+i;
            Vector2 position = nodes[i].Position;
            if(float.IsNaN(position.x) || float.IsNaN(position.y)) {
                throw new System.ArithmeticException("Node position is NaN:" + nodes[i].centerOffset.x + ", " + nodes[i].centerOffset.y + ", " + (nodes[i].pose == null ? "null" : nodes[i].pose.pose == null ? "pose"+nodes[i].pose.index+".null" : "p" + nodes[i].pose.pose.x + ", " + nodes[i].pose.pose.y));
            }
            circle.GameObj.transform.position = new Vector3(position.x, MAP_HEIGHT, position.y);
            circle.GameObj.transform.localScale = new Vector3(nodes[i].radius, ITEM_HEIGHT, nodes[i].radius);
            for(int k = 0; k < nodes[i].Connected.Count; k++) {
                if (nodes[i].Connected[k] > i) {
                    int[] c = new int[2];
                    c[0] = nodes[i].Connected[k];
                    c[1] = 1;
                    circle.Connected.Add(c);
                } else {
                    int[] c = new int[2];
                    c[0] = edges.Count;
                    c[1] = 0;
                    circle.Connected.Add(c);
                    circles[nodes[i].Connected[k]].Connected[circles[nodes[i].Connected[k]].Connected.FindIndex((int[] l) => { return l[0] == i; })][0] = edges.Count;
                    edges.Add(createEdge(nodes[nodes[i].Connected[k]].Position, nodes[i].Position, "E" + nodes[i].Connected[k] + "," + i));
                }
                
            }
            circles.Add(circle);
            i++;
        }
    }

    private LineRenderer createEdge(Vector2 a, Vector2 b, string name) {
        GameObject edge = Instantiate(prefabEdge, container);
        edge.name = name;
        edge.transform.parent = container;
        LineRenderer line = edge.GetComponent<LineRenderer>();
        Vector3[] positions = { new Vector3(a.x, MAP_HEIGHT, a.y), new Vector3(b.x, MAP_HEIGHT, b.y) };
        line.SetPositions(positions);
        return line;
    }

    public void RemoveEdge(int i, int j) {
        //Assert i < j
        var str = "E" + i + "," +j;
        edges.RemoveAll((LineRenderer r) => { return r.gameObject.name.Equals(str); });
    }
}
}