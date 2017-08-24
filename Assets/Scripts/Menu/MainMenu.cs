using ev3devMapping.Society;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using ev3devMapping.Testing;

namespace ev3devMapping {

public class MainMenu : MonoBehaviour {

    public const string BASE_SCENE = "Base";
    public const string MAIN_MENU_SCENE = "MainMenu";
    public const string TESTING_PLANNING_SCENE = "TestingPlaning";

    public const string SETTINGS = "settings.txt";
    public static readonly string[] INPUT_FIELDS = {
        "InputSessionDirectory",
        "InputPositionsKept",
        "InputWheelDiameter",
        "InputWheelBase",
        "InputTurningRadius",
        "InputDifferential",
        "InputPolarity",
        "InputLinearSpeed",
        "InputAngularSpeed",
        "InputOwnIP",
        "InputRobotIP"
    };

    public GameObject robotPrefab;

    public static RobotRequired Robot = null;
    public static Network Network = null;
    public static PositionHistory PositionHistory = null;
    public static Physics Physics = null;
    public static Limits Limits = null;

    public static bool IsNetworkingEnabled = false;

    public const int MAP_SLAM_ROBOT = 0;
    public const int MAP_GLOBAL_CLIENT = 1;

    protected virtual void Awake() {
        Network = gameObject.AddComponent<Network>();
        PositionHistory = gameObject.AddComponent<PositionHistory>();
        Physics = gameObject.AddComponent<Physics>();
        Limits = gameObject.AddComponent<Limits>();
        Robot = gameObject.AddComponent<RobotRequired>();
        DontDestroyOnLoad(gameObject);
        UnityEngine.SceneManagement.SceneManager.sceneLoaded += OnSceneLoaded;
    }

    public static bool TransferInput() {
        Robot.sessionDirectory = GameObject.Find(INPUT_FIELDS[0]).GetComponent<InputField>().text;
        if (!int.TryParse(GameObject.Find(INPUT_FIELDS[1]).GetComponent<InputField>().text, out PositionHistory.positionsKept)) {
            Debug.Log("PositionsKept is faulty.");
            return false;
        }
        if (!float.TryParse(GameObject.Find(INPUT_FIELDS[2]).GetComponent<InputField>().text, out Physics.wheelDiameterMm)) {
            Debug.Log("WheelDiameterMm is faulty.");
            return false;
        }
        if (!float.TryParse(GameObject.Find(INPUT_FIELDS[3]).GetComponent<InputField>().text, out Physics.wheelbaseMm)) {
            Debug.Log("WheelbaseMm is faulty.");
            return false;
        }
        if (!float.TryParse(GameObject.Find(INPUT_FIELDS[4]).GetComponent<InputField>().text, out Physics.turningRadius)) {
            Debug.Log("TurningRadius is faulty.");
            return false;
        }
        var differential = GameObject.Find(INPUT_FIELDS[5]).GetComponent<InputField>().text;
        if (differential.Equals("" + TachometerPosition.Differential)) Physics.Differential = TachometerPosition.Differential;
        else if (differential.Equals("" + TachometerPosition.Left)) Physics.Differential = TachometerPosition.Left;
        else if (differential.Equals("" + TachometerPosition.Right)) Physics.Differential = TachometerPosition.Right;
        else {
            Debug.Log("Differential is faulty.");
            return false;
        }
        if (!bool.TryParse(GameObject.Find(INPUT_FIELDS[6]).GetComponent<InputField>().text, out Physics.reverseMotorPolarity)) {
            Debug.Log("ReverseMotorPolarity is faulty.");
            return false;
        }
        if (!float.TryParse(GameObject.Find(INPUT_FIELDS[7]).GetComponent<InputField>().text, out Limits.MaxLinearSpeedMmPerSec)) {
            Debug.Log("MaxLinearSpeedMmPerSec is faulty.");
            return false;
        }
        if (!float.TryParse(GameObject.Find(INPUT_FIELDS[8]).GetComponent<InputField>().text, out Limits.MaxAngularSpeedDegPerSec)) {
            Debug.Log("MaxAngularSpeedDegPerSec is faulty.");
            return false;
        }
        Network.hostIp = GameObject.Find(INPUT_FIELDS[9]).GetComponent<InputField>().text;
        Network.robotIp = GameObject.Find(INPUT_FIELDS[10]).GetComponent<InputField>().text;

        if (File.Exists(SETTINGS)) File.Delete(SETTINGS);
        File.Create(SETTINGS).Dispose();
        using (var writer = new StreamWriter(File.OpenWrite(SETTINGS))) {
            for (int i = 0; i < INPUT_FIELDS.Length; i++) {
                writer.WriteLine(GameObject.Find(INPUT_FIELDS[i]).GetComponent<InputField>().text);
            }
            writer.Flush();
            writer.Close();
            writer.Dispose();
        }
        Directory.CreateDirectory(Robot.sessionDirectory);
        Physics.Calculate();
        return true;
    }

    void OnSceneLoaded(UnityEngine.SceneManagement.Scene scene, UnityEngine.SceneManagement.LoadSceneMode mode) {
        Debug.Log("Current scene: " + scene.name);
        if (scene.name.Equals(BASE_SCENE)) {
            if (!IsNetworkingEnabled) {
                //If the scene was loaded through the networking we don't have to call this code:
                GameObject r = Instantiate(robotPrefab, SceneManager.DynamicObjects);
                r.GetComponent<RobotNetworking>().OnStartLocalPlayer();
                return;
            }
        } else if(scene.name.Equals(MAIN_MENU_SCENE)) {
            IsNetworkingEnabled = false;
            if (!File.Exists(SETTINGS)) {
                GameObject.Find(INPUT_FIELDS[0]).GetComponent<InputField>().text = "" + Robot.sessionDirectory;
                GameObject.Find(INPUT_FIELDS[1]).GetComponent<InputField>().text = "" + PositionHistory.positionsKept;
                GameObject.Find(INPUT_FIELDS[2]).GetComponent<InputField>().text = "" + Physics.wheelDiameterMm;
                GameObject.Find(INPUT_FIELDS[3]).GetComponent<InputField>().text = "" + Physics.wheelbaseMm;
                GameObject.Find(INPUT_FIELDS[4]).GetComponent<InputField>().text = "" + Physics.turningRadius;
                GameObject.Find(INPUT_FIELDS[5]).GetComponent<InputField>().text = "" + Physics.Differential;
                GameObject.Find(INPUT_FIELDS[6]).GetComponent<InputField>().text = "" + Physics.reverseMotorPolarity;
                GameObject.Find(INPUT_FIELDS[7]).GetComponent<InputField>().text = "" + Limits.MaxLinearSpeedMmPerSec;
                GameObject.Find(INPUT_FIELDS[8]).GetComponent<InputField>().text = "" + Limits.MaxAngularSpeedDegPerSec;
                GameObject.Find(INPUT_FIELDS[9]).GetComponent<InputField>().text = "" + Network.hostIp;
                GameObject.Find(INPUT_FIELDS[10]).GetComponent<InputField>().text = "" + Network.robotIp;
            } else {
                using (var reader = new StreamReader(File.OpenRead(SETTINGS))) {
                    string s;
                    int i = 0;
                    while (!reader.EndOfStream && i < INPUT_FIELDS.Length) {
                        s = reader.ReadLine();
                        GameObject.Find(INPUT_FIELDS[i++]).GetComponent<InputField>().text = s;
                    }
                    reader.Close();
                    reader.Dispose();
                }
            }
        } else if(scene.name.Equals(TESTING_PLANNING_SCENE)) {
            using (var reader = new StreamReader(File.OpenRead(SETTINGS))) {
                Robot.sessionDirectory = reader.ReadLine();
                int.TryParse(reader.ReadLine(), out PositionHistory.positionsKept);
                float.TryParse(reader.ReadLine(), out Physics.wheelDiameterMm);
                float.TryParse(reader.ReadLine(), out Physics.wheelbaseMm);
                float.TryParse(reader.ReadLine(), out Physics.turningRadius);
                var differential = reader.ReadLine();
                if (differential.Equals("" + TachometerPosition.Differential)) Physics.Differential = TachometerPosition.Differential;
                else if (differential.Equals("" + TachometerPosition.Left)) Physics.Differential = TachometerPosition.Left;
                else Physics.Differential = TachometerPosition.Right;
                bool.TryParse(reader.ReadLine(), out Physics.reverseMotorPolarity);
                float.TryParse(reader.ReadLine(), out Limits.MaxLinearSpeedMmPerSec);
                float.TryParse(reader.ReadLine(), out Limits.MaxAngularSpeedDegPerSec);        
                Network.hostIp = reader.ReadLine();
                Network.robotIp = reader.ReadLine();
                reader.Close();
                reader.Dispose();
            }
            GameObject r = Instantiate(robotPrefab, SceneManager.DynamicObjects);
            //r.GetComponent<RobotNetworking>().OnStartLocalPlayer();
        }
    }
}
}
