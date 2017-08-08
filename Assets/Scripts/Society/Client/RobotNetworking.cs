using UnityEngine;
using UnityEngine.Networking;

public class RobotNetworking : NetworkBehaviour {

    public GameObject control;
    public GameObject deadReconning;
    public GameObject drive;
    //public GameObject laserXY;
    public GameObject laserXZ;
    public GameObject wifi;

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

        gameObject.AddComponent<Planing>();
        gameObject.AddComponent<SLAMRobot>();
    }

    public override void OnStartClient() {
        base.OnStartClient();
        //TODO: SET COLOR THROUGH COUNTER o.Ä. FROM SERVER
    }
}
