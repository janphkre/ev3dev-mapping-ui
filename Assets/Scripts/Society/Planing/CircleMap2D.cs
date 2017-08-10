using System.Collections.Generic;
using UnityEngine;

public class CircleMap2D : Object {

    public const float MAP_HEIGHT = 0.0f;
    public const float ITEM_HEIGHT = 0.01f;
    private GameObject prefab;
    private Transform container;
    private LinkedList<GameObject> circles = new LinkedList<GameObject>();

    public CircleMap2D(GameObject circle) {
        prefab = circle;
        container = new GameObject("CircleMap2D").transform;
        container.parent = SceneManager.DynamicObjects;
    }

    public void ProcessNodes(List<GraphNode> nodes, int count) {
        int i = 0;
        foreach(GameObject circle in circles) {
            Vector2 position = nodes[i++].Position;
            circle.transform.position = new Vector3(position.x, MAP_HEIGHT, position.y);
        }
        while(i < count) {
            GameObject circle = Instantiate(prefab, container);
            Vector2 position = nodes[i].Position;
            circle.transform.position = new Vector3(position.x, MAP_HEIGHT, position.y);
            circle.transform.localScale = new Vector3(nodes[i].radius, ITEM_HEIGHT, nodes[i].radius);
            circles.AddLast(circle);
            i++;
        }
    }
}
