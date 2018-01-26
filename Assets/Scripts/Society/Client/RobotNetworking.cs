using UnityEngine;
using UnityEngine.Networking;

namespace ev3devMapping.Society {

public class RobotNetworking : NetworkBehaviour {

    public GameObject control;
    public GameObject deadReconning;
    public GameObject drive;
    //public GameObject laserXY;
    public GameObject laserXZ;
    public GameObject wifi;
    public GameObject graph;

    //The local player will control the local robot:
    public override void OnStartLocalPlayer() {
        base.OnStartLocalPlayer();

        gameObject.name = "Robot";

        Network network = gameObject.AddComponent<Network>();
        network.copyFrom(MainMenu.Network);

        gameObject.AddComponent<Replay>();
        //Replay replay = gameObject.GetComponent<Replay>();

        PositionHistory positionHistory = gameObject.AddComponent<PositionHistory>();
        positionHistory.copyFrom(MainMenu.PositionHistory);

        Physics physics = gameObject.AddComponent<Physics>();
        physics.copyFrom(MainMenu.Physics);

        Limits limits = gameObject.AddComponent<Limits>();
        limits.copyFrom(MainMenu.Limits);

        gameObject.AddComponent<UserInput>();
        //UserInput userInput = gameObject.GetComponent<UserInput>();

        RobotRequired robotRequired = gameObject.AddComponent<RobotRequired>();
        robotRequired.copyFrom(MainMenu.Robot);

        gameObject.AddComponent<Map3D>();
        gameObject.AddComponent<Map3D>();

        gameObject.AddComponent<SLAMRobot>();

        var name = graph.name;
        graph = Instantiate(graph, gameObject.transform);
        graph.name = name;
        /*name = wifi.name;
        wifi = Instantiate(wifi, gameObject.transform);
        wifi.name = name;*/
        name = laserXZ.name;
        laserXZ = Instantiate(laserXZ, gameObject.transform);
        laserXZ.name = name;
        name = deadReconning.name;
        deadReconning = Instantiate(deadReconning, gameObject.transform);
        deadReconning.name = name;
        name = drive.name;
        drive = Instantiate(drive, gameObject.transform);
        drive.name = name;
        name = control.name;
        control = Instantiate(control, gameObject.transform);
        control.name = name;
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
}
