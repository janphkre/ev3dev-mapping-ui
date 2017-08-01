using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(RobotRequired))]
public class MainMenu : MonoBehaviour {

    public static RobotRequired Robot;
    public static Network Network;
    public static Replay Replay;
    public static PositionHistory PositionHistory;
    public static Physics Physics;
    public static Limits Limits;
    public static UserInput Input;

    protected virtual void Start() {
        Robot = gameObject.GetComponent<RobotRequired>();
        Network = gameObject.GetComponent<Network>();
        Replay = gameObject.GetComponent<Replay>();
        PositionHistory = gameObject.GetComponent<PositionHistory>();
        Physics = gameObject.GetComponent<Physics>();
        Limits = gameObject.GetComponent<Limits>();
        Input = gameObject.GetComponent<UserInput>();
        GameObject.Find("InputAcceleration").GetComponent<InputField>().text = "" + Input.accelerationPower;
        GameObject.Find("InputSessionDirectory").GetComponent<InputField>().text = "" + Robot.sessionDirectory;
        GameObject.Find("InputPositionsKept").GetComponent<InputField>().text = "" + PositionHistory.positionsKept;
        GameObject.Find("InputWheelDiameter").GetComponent<InputField>().text = "" + Physics.wheelDiameterMm;
        GameObject.Find("InputWheelBase").GetComponent<InputField>().text = "" + Physics.wheelbaseMm;
        GameObject.Find("InputEncoderRotation").GetComponent<InputField>().text = "" + Physics.encoderCountsPerRotation;
        GameObject.Find("InputEncoderMax").GetComponent<InputField>().text = "" + Physics.maxEncoderCountsPerSecond;
        GameObject.Find("InputPolarity").GetComponent<InputField>().text = "" + Physics.reverseMotorPolarity;
        GameObject.Find("InputLinearSpeed").GetComponent<InputField>().text = "" + Limits.MaxLinearSpeedMmPerSec;
        GameObject.Find("InputAngularSpeed").GetComponent<InputField>().text = "" + Limits.MaxAngularSpeedDegPerSec;
        GameObject.Find("InputOwnIP").GetComponent<InputField>().text = "" + Network.hostIp;
        GameObject.Find("InputRobotIP").GetComponent<InputField>().text = "" + Network.robotIp;
        //TODO:physics.maxTurningAngle
        //TODO:physics.innerTurningDiameter: 2 * MainMenu.Physics.TurningRadius - (MainMenu.Physics.wheelbaseMm / 1000.0f)
        //TODO: move input values into fields!
        DontDestroyOnLoad(gameObject);
    }

    public bool CheckInputs() {
        //TODO!
        return false;
    }

}