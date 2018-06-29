using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

public class CircleMap2D : Object {
    
    private struct Circle {
        public GameObject GameObj { get; private set; }
        public List<Edge> Edges {get; private set; }
        
        public Circle(GameObject gameObj) {
                GameObj = gameObj;
                Edges = new List<Edge>();
        }
    }

    private struct Edge {
        public LineRenderer Renderer { get; set; }
        public int ConnectedCircleIndex {get; private set; }
        public int RendererIndex {get; private set; }
        
        public Edge(LineRenderer renderer, int connectedCircleIndex, int rendererIndex) {
            Renderer = renderer;
            ConnectedCircleIndex = connectedCircleIndex;
            RendererIndex = rendererIndex;
        }
    }
    
    public const float MAP_HEIGHT = 0.0f;
    public const float MAP_HEIGHT_2 = 0.05f;
    public const float ITEM_HEIGHT = 0.001f;
    private GameObject prefabCircle;
    private GameObject prefabEdge;
    private Transform container;
    private List<Circle> circles = new List<Circle>();
    
    public CircleMap2D(GameObject circle, GameObject edge) {
        prefabCircle = circle;
        prefabEdge = edge;
        container = new GameObject("CircleMap2D").transform;
        container.parent = SceneManager.DynamicObjects;
    }

    public void ProcessNodes(List<GraphNode> nodes, List<int> unvisitedNodes) {
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
            foreach (Edge edge in circle.Edges) {
                //circle.Connected[i] = nodes[i].Connected[k];
                edge.Renderer.SetPosition(edge.RendererIndex, circle.GameObj.transform.position);
            }
            createEdges(nodes, i, circle);
            i++;
        }
        while(i < nodes.Count) {
            //New nodes:
            Circle circle = new Circle(Instantiate(prefabCircle, container));
            circle.GameObj.name = "C"+i;
            Vector2 position = nodes[i].Position;
            if(float.IsNaN(position.x) || float.IsNaN(position.y)) {
                throw new System.ArithmeticException("Node position is NaN:" + nodes[i].centerOffset.x + ", " + nodes[i].centerOffset.y + ", " + (nodes[i].pose == null ? "null" : nodes[i].pose.pose == null ? "pose"+nodes[i].pose.index+".null" : "p" + nodes[i].pose.pose.x + ", " + nodes[i].pose.pose.y));
            }
            circle.GameObj.transform.position = new Vector3(position.x, MAP_HEIGHT, position.y);
            circle.GameObj.transform.localScale = new Vector3(2*nodes[i].radius, ITEM_HEIGHT, 2*nodes[i].radius);
            createEdges(nodes, i, circle);
            circles.Add(circle);
            i++;
        }
    }

    private void createEdges(List<GraphNode> nodes, int currentNodeIndex, Circle circle) {
        var currentNode = nodes[currentNodeIndex];
        for(int k = circle.Edges.Count; k < currentNode.Connected.Count; k++) {
            var connectedNodeIndex = currentNode.Connected[k];
            if (currentNode.Connected[k] < currentNodeIndex) {
                //Connected Node has been visited already.
                //Create a new Edge since we know that it exists.
                var edge = new Edge(
                    createEdge(nodes[connectedNodeIndex].Position, currentNode.Position, "E" + connectedNodeIndex + "," + currentNodeIndex),
                    connectedNodeIndex,
                    1);
                circle.Edges.Add(edge);
                circles[connectedNodeIndex].Edges.Add(new Edge(
                    edge.Renderer,
                    currentNodeIndex,
                    0));
            }
            // else Connected Node is yet to be created / processed.
        }
        }

    private LineRenderer createEdge(Vector2 a, Vector2 b, string name) {
        GameObject edge = Instantiate(prefabEdge, container);
        edge.name = name;
        edge.transform.parent = container;
        LineRenderer line = edge.GetComponent<LineRenderer>();
        Vector3[] positions = { new Vector3(a.x, MAP_HEIGHT_2, a.y), new Vector3(b.x, MAP_HEIGHT_2, b.y) };
        line.SetPositions(positions);
        return line;
    }

    public void RemoveEdge(int i, int j) {
        circles[i].Edges.RemoveAll((Edge e) => { return e.ConnectedCircleIndex == j; });
        circles[j].Edges.RemoveAll((Edge e) => { return e.ConnectedCircleIndex == i; });
    }
}
}