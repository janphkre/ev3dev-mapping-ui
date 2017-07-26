using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(RobotRequired))]
public class MainMenu : MonoBehaviour {

    private RobotRequired robot;
    private Network network;
    private Replay replay;
    private PositionHistory positionHistory;
    private Physics physics;
    private Limits limits;
    private UserInput input;

    protected virtual void Start() {
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
        //Make Networ
        //TODO: move input values into fields!
        DontDestroyOnLoad(gameObject);
    }

    public bool CheckInputs() {
        //TODO!
        return false;
    }

    public RobotRequired GetRobotRequired() {
        return robot;
    }

    public Network GetNetwork() {
        return network;
    }

    public Replay GetReplay() {
        return replay;
    }

    public PositionHistory GetPositionHistory() {
        return positionHistory;
    }

    public Physics GetPhysics() {
        return physics;
    }

    public Limits GetLimits() {
        return limits;
    }

    public UserInput GetUserInput() {
        return input;
    }

}