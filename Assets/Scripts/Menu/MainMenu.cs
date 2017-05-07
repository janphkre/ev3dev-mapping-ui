using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;

[RequireComponent(typeof(RobotRequired))]
public class MainMenu : MonoBehaviour {

    private NetworkManager society;
    private RobotRequired robot;
    private Network network;
    private Replay replay;
    private PositionHistory positionHistory;
    private Physics physics;
    private Limits limits;
    private UserInput input;

    protected virtual void Start() {
        society = GameObject.Find("Society").GetComponent<NetworkManager>();
        robot = gameObject.GetComponent<RobotRequired>();
        network = gameObject.GetComponent<Network>();
        replay = gameObject.GetComponent<Replay>();
        positionHistory = gameObject.GetComponent<PositionHistory>();
        physics = gameObject.GetComponent<Physics>();
        limits = gameObject.GetComponent<Limits>();
        input = gameObject.GetComponent<UserInput>();
        GameObject.Find("InputAcceleration").GetComponent<InputField>().text = "" + input.accelerationPower;
        GameObject.Find("InputSessionDirectory").GetComponent<InputField>().text = "" + robot.sessionDirectory;
        GameObject.Find("InputPositionsKept").GetComponent<InputField>().text = "" + positionHistory.positionsKept;
        GameObject.Find("InputWheelDiameter").GetComponent<InputField>().text = "" + physics.wheelDiameterMm;
        GameObject.Find("InputWheelBase").GetComponent<InputField>().text = "" + physics.wheelbaseMm;
        GameObject.Find("InputEncoderRotation").GetComponent<InputField>().text = "" + physics.encoderCountsPerRotation;
        GameObject.Find("InputEncoderMax").GetComponent<InputField>().text = "" + physics.maxEncoderCountsPerSecond;
        GameObject.Find("InputPolarity").GetComponent<InputField>().text = "" + physics.reverseMotorPolarity;
        GameObject.Find("InputLinearSpeed").GetComponent<InputField>().text = "" + limits.MaxLinearSpeedMmPerSec;
        GameObject.Find("InputAngularSpeed").GetComponent<InputField>().text = "" + limits.MaxAngularSpeedDegPerSec;
        GameObject.Find("InputOwnIP").GetComponent<InputField>().text = "" + network.hostIp;
        GameObject.Find("InputRobotIP").GetComponent<InputField>().text = "" + network.robotIp;

        DontDestroyOnLoad(gameObject);
    }

    public RobotRequired getRobotRequired() {
        return robot;
    }

    public Network getNetwork() {
        return network;
    }

    public Replay getReplay() {
        return replay;
    }

    public PositionHistory getPositionHistory() {
        return positionHistory;
    }

    public Physics getPhysics() {
        return physics;
    }

    public Limits getLimits() {
        return limits;
    }

    public UserInput getUserInput() {
        return input;
    }

}