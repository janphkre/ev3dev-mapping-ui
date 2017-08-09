using UnityEngine;
using UnityEngine.UI;
using System.IO;

[RequireComponent(typeof(RobotRequired))]
public class MainMenu : MonoBehaviour {

    public const string SETTINGS = "settings";
    public static readonly string[] INPUT_FIELDS = {
        "InputSessionDirectory",
        "InputPositionsKept",
        "InputWheelDiameter",
        "InputTurningRadius",
        "InputDifferential",
        "InputPolarity",
        "InputLinearSpeed",
        "InputAngularSpeed",
        "InputOwnIP",
        "InputRobotIP"
    };
    public static RobotRequired Robot;
    public static Network Network;
    public static PositionHistory PositionHistory;
    public static Physics Physics;
    public static Limits Limits;

    public const int MAP_SLAM_ROBOT = 0;
    public const int MAP_GLOBAL_CLIENT = 1;

    protected virtual void Start() {
        Robot = gameObject.GetComponent<RobotRequired>();
        Network = gameObject.GetComponent<Network>();
        PositionHistory = gameObject.GetComponent<PositionHistory>();
        Physics = gameObject.GetComponent<Physics>();
        Limits = gameObject.GetComponent<Limits>();
        if (!File.Exists(SETTINGS)) {
            GameObject.Find(INPUT_FIELDS[0]).GetComponent<InputField>().text = "" + Robot.sessionDirectory;
            GameObject.Find(INPUT_FIELDS[1]).GetComponent<InputField>().text = "" + PositionHistory.positionsKept;
            GameObject.Find(INPUT_FIELDS[2]).GetComponent<InputField>().text = "" + Physics.wheelDiameterMm;
            GameObject.Find(INPUT_FIELDS[3]).GetComponent<InputField>().text = "" + Physics.wheelbaseMm;
            GameObject.Find(INPUT_FIELDS[4]).GetComponent<InputField>().text = "" + Physics.turningRadius;
            GameObject.Find(INPUT_FIELDS[5]).GetComponent<InputField>().text = "" + Physics.hasDifferential;
            GameObject.Find(INPUT_FIELDS[6]).GetComponent<InputField>().text = "" + Physics.reverseMotorPolarity;
            GameObject.Find(INPUT_FIELDS[7]).GetComponent<InputField>().text = "" + Limits.MaxLinearSpeedMmPerSec;
            GameObject.Find(INPUT_FIELDS[8]).GetComponent<InputField>().text = "" + Limits.MaxAngularSpeedDegPerSec;
            GameObject.Find(INPUT_FIELDS[9]).GetComponent<InputField>().text = "" + Network.hostIp;
            GameObject.Find(INPUT_FIELDS[10]).GetComponent<InputField>().text = "" + Network.robotIp;
        } else {
            var reader = new StreamReader(File.OpenRead(SETTINGS));
            string s;
            int i = 0;
            while ((s = reader.ReadLine()) != null && i < INPUT_FIELDS.Length) {
                GameObject.Find(INPUT_FIELDS[i++]).GetComponent<InputField>().text = s;
            }
            reader.Close();
        }
        DontDestroyOnLoad(gameObject);
    }

    public static bool TransferInput() {
        Robot.sessionDirectory = GameObject.Find(INPUT_FIELDS[0]).GetComponent<InputField>().text;
        if (int.TryParse(GameObject.Find(INPUT_FIELDS[1]).GetComponent<InputField>().text, out PositionHistory.positionsKept)) return false;
        if (float.TryParse(GameObject.Find(INPUT_FIELDS[2]).GetComponent<InputField>().text, out Physics.wheelDiameterMm)) return false;
        if (float.TryParse(GameObject.Find(INPUT_FIELDS[3]).GetComponent<InputField>().text, out Physics.wheelbaseMm)) return false;
        if (float.TryParse(GameObject.Find(INPUT_FIELDS[4]).GetComponent<InputField>().text, out Physics.turningRadius)) return false;
        if (bool.TryParse(GameObject.Find(INPUT_FIELDS[5]).GetComponent<InputField>().text, out Physics.hasDifferential)) return false;
        if (bool.TryParse(GameObject.Find(INPUT_FIELDS[6]).GetComponent<InputField>().text, out Physics.reverseMotorPolarity)) return false;        
        if (float.TryParse(GameObject.Find(INPUT_FIELDS[7]).GetComponent<InputField>().text, out Limits.MaxLinearSpeedMmPerSec)) return false;
        if (float.TryParse(GameObject.Find(INPUT_FIELDS[8]).GetComponent<InputField>().text, out Limits.MaxAngularSpeedDegPerSec)) return false;
        Network.hostIp = GameObject.Find(INPUT_FIELDS[9]).GetComponent<InputField>().text;
        Network.robotIp = GameObject.Find(INPUT_FIELDS[10]).GetComponent<InputField>().text;

        if (File.Exists(SETTINGS)) File.Delete(SETTINGS);
        File.Create(SETTINGS);
        var writer = new StreamWriter(File.OpenWrite(SETTINGS));
        for(int i = 0; i < INPUT_FIELDS.Length; i++) {
            writer.WriteLine(GameObject.Find(INPUT_FIELDS[i]).GetComponent<InputField>().text);
        }
        writer.Flush();
        writer.Close();
        Directory.CreateDirectory(Robot.sessionDirectory);
        Physics.Calculate();
        return true;
    }

}