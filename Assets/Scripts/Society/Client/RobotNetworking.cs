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
        network.copyFrom(mainMenu.getNetwork());

        gameObject.AddComponent<Replay>();
        Replay replay = gameObject.GetComponent<Replay>();
        replay.copyFrom(mainMenu.getReplay());

        gameObject.AddComponent<PositionHistory>();
        PositionHistory positionHistory = gameObject.GetComponent<PositionHistory>();
        positionHistory.copyFrom(mainMenu.getPositionHistory());

        gameObject.AddComponent<Physics>();
        Physics physics = gameObject.GetComponent<Physics>();
        physics.copyFrom(mainMenu.getPhysics());

        gameObject.AddComponent<Limits>();
        Limits limits = gameObject.GetComponent<Limits>();
        limits.copyFrom(mainMenu.getLimits());

        gameObject.AddComponent<UserInput>();
        UserInput userInput = gameObject.GetComponent<UserInput>();
        userInput.copyFrom(mainMenu.getUserInput());

        gameObject.AddComponent<RobotRequired>();
        RobotRequired robotRequired = gameObject.GetComponent<RobotRequired>();
        robotRequired.copyFrom(mainMenu.getRobotRequired());

        GameObject obj = (GameObject) Instantiate(control, gameObject.transform);
        obj = (GameObject) Instantiate(deadReconning, gameObject.transform);
        obj = (GameObject) Instantiate(drive, gameObject.transform);
        obj = (GameObject) Instantiate(laserXY, gameObject.transform);
        obj = (GameObject) Instantiate(laserXZ, gameObject.transform);
        obj = (GameObject) Instantiate(wifi, gameObject.transform);
    }

    public override void OnStartClient() {
        base.OnStartClient();
        //TODO: SET COLOR THROUGH COUNTER o.Ä. FROM SERVER
    }
}
