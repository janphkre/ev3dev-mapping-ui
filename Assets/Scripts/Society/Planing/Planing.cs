using System;
using System.Collections;
using System.IO;
using ev3dev.Society;
using UnityEngine;

namespace ev3devMapping.Society {

public class PlaningInputData {
    
    public static float MIN_READING_DISTANCE = 0.02f;

    public Vector3 LastPose;
    public Vector3[] Readings;
    public Vector2[] ReadingsRB;
    public int ReadingsCount { get; internal set; }
    
    public PlaningInputData() { }

    public PlaningInputData(Vector3[] readings, bool[] invalid, int invalidCount) {
        Readings = new Vector3[readings.Length - invalidCount];
        ReadingsRB = new Vector2[readings.Length - invalidCount];
        int count = 0;
            for (int i = 0; i < readings.Length; i++) {
                if (!invalid[i]) Readings[count++] = readings[i];
        }
        ReadingsCount = count;
    }

    public void CalculateRB(PositionData pos) {
        LastPose = new Vector3(pos.position.x, pos.position.z, Geometry.angleToCircle(pos.heading * Mathf.PI / 180f));
        CalculateRBForPose();
    }
    
    #region Testing

    private const string PLANING_FILE = "Planing_Test.txt";

    public void Write() {
        if (File.Exists(PLANING_FILE)) File.Delete(PLANING_FILE);
        File.Create(PLANING_FILE).Dispose();
        using (var writer = new StreamWriter(File.OpenWrite(PLANING_FILE))) {
            writer.WriteLine(LastPose.ToString());
            writer.WriteLine(ReadingsCount);
            for (int i = 0; i < ReadingsCount; i++) {
                writer.WriteLine(Readings[i].ToString());
            }
            writer.Flush();
            writer.Close();
            writer.Dispose();
        }
    }

    public void Read() {
        if (!File.Exists(PLANING_FILE)) Debug.Log(PLANING_FILE + " does not exist!");
        using (var reader = new StreamReader(File.OpenRead(PLANING_FILE))) {
            LastPose = V3Parse(reader.ReadLine());
            int i = 0;
            ReadingsCount = int.Parse(reader.ReadLine());
            Readings = new Vector3[ReadingsCount];
            ReadingsRB = new Vector2[ReadingsCount];
            while (!reader.EndOfStream && i < ReadingsCount) {
                Readings[i] = V3Parse(reader.ReadLine());
                i++;
            }
            reader.Close();
            reader.Dispose();
        }
        CalculateRBForPose();
    }

    private void CalculateRBForPose() {
        int j = 0;
        for (int i = 0; i < ReadingsCount; i++) {
            var rb = Geometry.ToRangeBearing2(Readings[i], LastPose);
            if (rb.x < MIN_READING_DISTANCE) continue;
            ReadingsRB[j] = rb;
            Readings[j] = Readings[i];
            j++;
        }
        ReadingsCount = j;
    }

    private Vector3 V3Parse(string s) {
        string[] r = s.Split(new char[] { '(', ',', ' ', ')' });
        return new Vector3(float.Parse(r[1]), float.Parse(r[3]), float.Parse(r[5]));
    }

    private void V2Parse(string s, out Vector3 v, out Vector2 w) {
        string[] r = s.Split(new char[] { '(', ',', ' ', ')' });
        v = new Vector3(float.Parse(r[1]), float.Parse(r[3]), float.Parse(r[5]));
        w = new Vector2(float.Parse(r[8]), float.Parse(r[10]));
    }
    #endregion
}

[RequireComponent (typeof(Graph))]
[RequireComponent (typeof(PlaningUI))]
public class Planing : MonoBehaviour {

    public static Planing singleton;

    public PlaningInputData LaserReadings {
            private get { lock (laserReadingsLock) return currentLaserReadings; }
            set { lock (laserReadingsLock) currentLaserReadings = value; }
    }

    public Graph GlobalGraph { get { return algorithms.globalGraph; } }

    private CarDrive steering;
    private PositionHistory positionHistory;

    private PlaningAlgorithms algorithms = new PlaningAlgorithms();
    private TargetCommandWrapper commandWrapper;
    private TurnObserver turnObserver;
    private object laserReadingsLock = new object();
    private volatile PlaningInputData currentLaserReadings = null;

    public void Awake() {
        singleton = this;
    }

    public void Start() {
        SetupAlgorithms();
        StartCoroutine("turnRoutine");
        StartCoroutine("planingRoutine");
    }

    public void StartPlaning() {
        commandWrapper.SetStartCommand();
    }

    public void ReturnToStart() {
        commandWrapper.SetReturnToStartCommand();
    }

    private void SetupAlgorithms() {
            AbstractTargetCommand.UNOBSTRUCTED_OFFSET = Mathf.Acos(1f - AbstractTargetCommand.UNOBSTRUCTED_OBSTACLE_MULTIPLIER * AbstractTargetCommand.MIN_OBSTACLE_DISTANCE / MainMenu.Physics.turningRadius);

            steering = transform.parent.gameObject.GetComponentInChildren<CarDrive>();
            positionHistory = transform.parent.gameObject.GetComponent<PositionHistory>();

            turnObserver = new TurnObserver(steering, positionHistory);

            GameObject obj = new GameObject("RendererX");
            algorithms.rendererX = obj.AddComponent<LineRenderer>();
            algorithms.rendererX.startWidth = 0.01f;
            algorithms.rendererX.endWidth = 0.01f;
            algorithms.rendererX.positionCount = 0;
            obj = new GameObject("RendererY");
            algorithms.rendererY = obj.AddComponent<LineRenderer>();
            algorithms.rendererY.startWidth = 0.01f;
            algorithms.rendererY.endWidth = 0.01f;
            algorithms.rendererY.positionCount = 0;
            obj = new GameObject("RendererSteer");
            algorithms.rendererSteering = obj.AddComponent<LineRenderer>();
            algorithms.rendererSteering.material.color = Color.green;
            algorithms.rendererSteering.startWidth = 0.02f;
            algorithms.rendererSteering.endWidth = 0.02f;
            algorithms.rendererSteering.positionCount = 0;

            algorithms.rendererPositiveTurningCenter = GameObject.CreatePrimitive(PrimitiveType.Cube);
            algorithms.rendererPositiveTurningCenter.transform.localScale = new Vector3(0.05f, 0.05f, 0.05f);
            algorithms.rendererPositiveTurningCenter.GetComponent<MeshRenderer>().material.color = Color.magenta;

            algorithms.globalGraph = gameObject.GetComponent<Graph>();
            algorithms.steering = turnObserver;

            commandWrapper = new TargetCommandWrapper(algorithms);
    }

    private IEnumerator planingRoutine() {
        while (true) {
            yield return new WaitWhile(() => algorithms.lastLaserReadings == currentLaserReadings);
            algorithms.lastLaserReadings = LaserReadings;
            algorithms.lastLaserReadings.CalculateRB(positionHistory.GetNewestThreadSafe());
            algorithms.lastLaserReadings.Write();
            commandWrapper.ExecuteStep();
        }
    }

    private IEnumerator turnRoutine() {
        return turnObserver.turnRoutine();
    }

    public String GetCurrentTargetString() {
        return commandWrapper.GetCurrentTargetCommand().ToString();
    }
}
}