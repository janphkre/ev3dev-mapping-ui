using UnityEngine;
using UnityEngine.Networking;

public class RobotNetworking : NetworkBehaviour {

    public GameObject control;
    public GameObject deadReconning;
    public GameObject drive;
    public GameObject laserXY;
    public GameObject laserXZ;
    public GameObject wifi;

    //The local player will control the local robot:
    public override void OnStartLocalPlayer() {
        base.OnStartLocalPlayer();
        MainMenu mainMenu = GameObject.Find("Settings").GetComponent<MainMenu>();

        gameObject.AddComponent<Network>();
        Network network = gameObject.GetComponent<Network>();
        network.copyFrom(mainMenu.GetNetwork());

        gameObject.AddComponent<Replay>();
        Replay replay = gameObject.GetComponent<Replay>();
        replay.copyFrom(mainMenu.GetReplay());

        gameObject.AddComponent<PositionHistory>();
        PositionHistory positionHistory = gameObject.GetComponent<PositionHistory>();
        positionHistory.copyFrom(mainMenu.GetPositionHistory());

        gameObject.AddComponent<Physics>();
        Physics physics = gameObject.GetComponent<Physics>();
        physics.copyFrom(mainMenu.GetPhysics());

        gameObject.AddComponent<Limits>();
        Limits limits = gameObject.GetComponent<Limits>();
        limits.copyFrom(mainMenu.GetLimits());

        gameObject.AddComponent<UserInput>();
        UserInput userInput = gameObject.GetComponent<UserInput>();
        userInput.copyFrom(mainMenu.GetUserInput());

        gameObject.AddComponent<RobotRequired>();
        RobotRequired robotRequired = gameObject.GetComponent<RobotRequired>();
        robotRequired.copyFrom(mainMenu.GetRobotRequired());

        /*var obj = Instantiate(control, gameObject.transform);
        obj = Instantiate(deadReconning, gameObject.transform);
        obj = Instantiate(drive, gameObject.transform);
        obj = Instantiate(laserXY, gameObject.transform);
        obj = Instantiate(laserXZ, gameObject.transform);
        obj = Instantiate(wifi, gameObject.transform);*/
    }

    public override void OnStartClient() {
        base.OnStartClient();
        //TODO: SET COLOR THROUGH COUNTER o.Ä. FROM SERVER
    }
}
