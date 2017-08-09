using UnityEngine;
using UnityEngine.Networking;

public class RobotNetworking : NetworkBehaviour {

    public GameObject control;
    public GameObject deadReconning;
    public GameObject drive;
    //public GameObject laserXY;
    public GameObject laserXZ;
    public GameObject wifi;
    public GameObject circle;

    //The local player will control the local robot:
    public override void OnStartLocalPlayer() {
        base.OnStartLocalPlayer();

        gameObject.AddComponent<Network>();
        Network network = gameObject.GetComponent<Network>();
        network.copyFrom(MainMenu.Network);

        gameObject.AddComponent<Replay>();
        //Replay replay = gameObject.GetComponent<Replay>();

        gameObject.AddComponent<PositionHistory>();
        PositionHistory positionHistory = gameObject.GetComponent<PositionHistory>();
        positionHistory.copyFrom(MainMenu.PositionHistory);

        gameObject.AddComponent<Physics>();
        Physics physics = gameObject.GetComponent<Physics>();
        physics.copyFrom(MainMenu.Physics);

        gameObject.AddComponent<Limits>();
        Limits limits = gameObject.GetComponent<Limits>();
        limits.copyFrom(MainMenu.Limits);

        gameObject.AddComponent<UserInput>();
        //UserInput userInput = gameObject.GetComponent<UserInput>();

        gameObject.AddComponent<RobotRequired>();
        RobotRequired robotRequired = gameObject.GetComponent<RobotRequired>();
        robotRequired.copyFrom(MainMenu.Robot);

        gameObject.AddComponent<Graph>();
        Graph graph = gameObject.GetComponent<Graph>();
        graph.Map = new CircleMap2D(circle);

        gameObject.AddComponent<Map3D>();
        gameObject.AddComponent<Map3D>();

        gameObject.AddComponent<Planing>();
        gameObject.AddComponent<SLAMRobot>();
    }

    public override void OnStartClient() {
        base.OnStartClient();
        NetworkManager.singleton.client.RegisterHandler((short)MessageType.Color, OnColor);
        NetworkManager.singleton.client.Send((short) MessageType.ColorRequest, new RequestMessage());
    }

    public void OnColor(NetworkMessage netMsg) {
        Debug.Log("Recieved color message from the server.");
        ColorMessage msg = netMsg.ReadMessage<ColorMessage>();
        Material material = gameObject.GetComponentInChildren<Material>();
        material.color = msg.color;
    }
}
