using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace ev3devMapping.Society {

public enum TargetCommand {
    RandomMove,
    ExplorePosition,
    Backtrack,
    Turn,
    Waiting
}

public class PlaningInputData {
    
    public static float MIN_READING_DISTANCE = 0.04f;

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
        LastPose = new Vector3(pos.position.x, pos.position.z, pos.heading * Mathf.PI / 180f);
        int j = 0;
        for (int i = 0; i < ReadingsCount; i++) {
            var rb = Geometry.ToRangeBearing2(Readings[i], LastPose);
            if(rb.x < MIN_READING_DISTANCE) continue;
            ReadingsRB[j] = rb;
            Readings[j] = Readings[i];
            j++;
        }
        ReadingsCount = j;
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
                writer.WriteLine(Readings[i].ToString() + "; " + ReadingsRB[i].ToString());
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
                //V2Parse(reader.ReadLine(), out Readings[i], out ReadingsRB[i]);
                Readings[i] = V3Parse(reader.ReadLine());
                /*if(ReadingsRB[i].y > Geometry.HALF_CIRCLE) {
                    ReadingsRB[i].y -= Geometry.FULL_CIRCLE;
                } else if(ReadingsRB[i].y < -Geometry.HALF_CIRCLE) {
                    ReadingsRB[i].y += Geometry.FULL_CIRCLE;
                }*/
                i++;
            }
            reader.Close();
            reader.Dispose();
        }
        CalculateRB(new PositionData());
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

    //Parameters:
    public const float ALPHA = Geometry.HALF_CIRCLE + Geometry.HALF_CIRCLE / 4f;
    public const float BETA = Geometry.RIGHT_ANGLE + Geometry.RIGHT_ANGLE / 3f;
    public const float GAMMA = Geometry.RIGHT_ANGLE / 8f;
    public const float OBSTACLE_PLANING_STEP = Geometry.HALF_CIRCLE / 36f;
    public const float MIN_OBSTACLE_DISTANCE = 0.1f;
    public const float UNOBSTRUCTED_OBSTACLE_MULTIPLIER = 1f;
    public const float TARGET_RADIUS = 0.1f;
    public const float MAX_OFFSET_ANGLE = Geometry.HALF_CIRCLE;
    public const float MIN_CORRECTION_ANGLE = Geometry.HALF_CIRCLE / 180f;
    public const float ARC_STEP = 1f / 4f;
    public const float UNOBSTRUCTED_HEIGHT = 0.02f;
    public const float STEERING_HEIGHT = 0.04f;
    
    //Calculated once at Startup:
    public static float UNOBSTRUCTED_OFFSET;

    public static Planing singleton;

    public PlaningInputData LaserReadings {
        private get { lock (laserReadingsLock) return currentLaserReadings; }
        set { /*Debug.Log("New LaserReading.");*/ lock (laserReadingsLock) currentLaserReadings = value; }
    }

    public Graph GlobalGraph { get { return globalGraph; } }

    private Graph globalGraph;
    private CarDrive steering;
    private PositionHistory positionHistory;
    private Stack<TargetCommand> currentTarget = new Stack<TargetCommand>();
    private Vector2 currentTargetPosition = new Vector2(1f, 0f);
    private volatile bool returnToStart = false;
    private volatile bool start = false;
    private LinkedList<Vector2> currentPath;

    private object laserReadingsLock = new object();
    private volatile PlaningInputData currentLaserReadings = null;
    private PlaningInputData lastLaserReadings = null;

    private bool backwards = false;
    private Vector3 positiveTurningCenter;//LEFT
    private Vector3 negativeTurningCenter;//RIGHT
    private List<Vector3> obstacles;
    
    private LineRenderer rendererX;
    private LineRenderer rendererY;
    private LineRenderer rendererSteering;

    public void Awake() {
        singleton = this;
        UNOBSTRUCTED_OFFSET = Mathf.Acos(1f - UNOBSTRUCTED_OBSTACLE_MULTIPLIER * MIN_OBSTACLE_DISTANCE / MainMenu.Physics.turningRadius);
        globalGraph = gameObject.GetComponent<Graph>();
        currentTarget.Push(TargetCommand.Waiting);

        GameObject obj = new GameObject("RendererX");
        rendererX = obj.AddComponent<LineRenderer>();
        rendererX.startWidth = 0.01f;
        rendererX.endWidth = 0.01f;
        rendererX.positionCount = 0;
        obj = new GameObject("RendererY");
        rendererY = obj.AddComponent<LineRenderer>();
        rendererY.startWidth = 0.01f;
        rendererY.endWidth = 0.01f;
        rendererY.positionCount = 0;
        obj = new GameObject("RendererSteer");
        rendererSteering = obj.AddComponent<LineRenderer>();
        rendererSteering.material.color = Color.green;
        rendererSteering.startWidth = 0.02f;
        rendererSteering.endWidth = 0.02f;
        rendererSteering.positionCount = 0;
    }

    public void Start() {
        steering = transform.parent.gameObject.GetComponentInChildren<CarDrive>();
        positionHistory = transform.parent.gameObject.GetComponent<PositionHistory>();
        StartCoroutine("workerRoutine");
    }

    public TargetCommand GetCurrentTarget() {
        lock (currentTarget) return currentTarget.Peek();
    }

    public Vector2 GetCurrentTargetPosition() {
        return currentTargetPosition;
    }

    public void StartPlaning() {
        start = true;
    }

    public void ReturnToStart() {
        returnToStart = true;
    }

    public void Offroad() {
        
    }

    private IEnumerator workerRoutine() {
        bool wasUsed = true;
        while (true) {
            if (wasUsed) yield return new WaitWhile(() => lastLaserReadings == currentLaserReadings);
            lastLaserReadings = LaserReadings;
            wasUsed = false;
            if (currentTarget.Peek() == TargetCommand.ExplorePosition) {
                //Debug.Log("Planing - Exploring position.");
                lastLaserReadings.CalculateRB(positionHistory.GetNewestThreadSafe());
                if (obstaclePlaning()) {
                    wasUsed = true;
                    globalGraph.Feed(lastLaserReadings);
                }
            /*} else if (currentTarget.Peek() == TargetCommand.Turn) {
                //Debug.Log("Planing - Turning.");
                //Wait for the turn to finish:
                yield return new WaitWhile(steering.IsTurning);
                wasUsed = true;
                lock (currentTarget) currentTarget.Pop();*/
            } else if (currentTarget.Peek() == TargetCommand.Waiting) {
                if (returnToStart) {
                    //Debug.Log("Planing - Returning to start.");
                    steering.Halt();
                    lock (currentTarget) {
                        currentTarget.Clear();
                        currentTarget.Push(TargetCommand.Waiting);
                        currentTarget.Push(TargetCommand.Backtrack);
                    }
                    currentPath = globalGraph.GetStartPath(lastLaserReadings.LastPose);
                    currentTargetPosition = currentPath.First.Value;
                    currentPath.RemoveFirst();
                    backwards = !backwards;
                    returnToStart = false;
                } else if (start) {
                    lock (currentTarget) { currentTarget.Push(TargetCommand.RandomMove); }
                    start = false;
                    lastLaserReadings.CalculateRB(positionHistory.GetNewestThreadSafe());
                    lastLaserReadings.Write();
                } else {
                    lastLaserReadings.CalculateRB(positionHistory.GetNewestThreadSafe());
                    globalGraph.Feed(lastLaserReadings);
                    wasUsed = true;
                }
            } else {
                bool calculating;
                do {
                    calculating = false;
                    if (currentTarget.Peek() == TargetCommand.RandomMove) {
                        //Debug.Log("Planing - Calculating random move.");
                        if (!globalGraph.GetNewTarget(out currentTargetPosition)) {
                            steering.Halt();
                            if (globalGraph.HasUnvisitedNodes()) {
                                lock (currentTarget) currentTarget.Push(TargetCommand.Backtrack);
                                //Backtrack as we don't have any target left at the moment.
                                calculating = true;
                            } else {
                                //The graph does not provide any more unvisited nodes: stop and wait.
                                lock (currentTarget) currentTarget.Pop();
                                calculating = true;
                            }
                        } else lock (currentTarget) currentTarget.Push(TargetCommand.ExplorePosition);
                    } else if (currentTarget.Peek() == TargetCommand.ExplorePosition) {
                        //Debug.Log("Planing - Calculating explore position.");
                        if (currentPath.Count <= 0) {
                            currentPath = null;
                            lock (currentTarget) currentTarget.Pop();
                            calculating = true;
                        } else {
                            currentTargetPosition = currentPath.First.Value;
                            currentPath.RemoveFirst();
                        }
                    } else if (currentTarget.Peek() == TargetCommand.Backtrack) {
                        //Debug.Log("Planing - Calculating backtrack.");
                        if (currentPath == null) {
                            currentPath = globalGraph.GetUnexploredNodePath(lastLaserReadings.LastPose);
                            currentTargetPosition = currentPath.First.Value;
                            currentPath.RemoveFirst();
                            backwards = !backwards;
                            lock (currentTarget) currentTarget.Push(TargetCommand.ExplorePosition);
                        } else if (currentPath.Count == 0) {
                            lock (currentTarget) currentTarget.Pop();
                            calculating = true;
                        } else {
                            lock (currentTarget) currentTarget.Push(TargetCommand.ExplorePosition);
                            currentTargetPosition = currentPath.First.Value;
                            currentPath.RemoveFirst();
                        }
                    }
                } while (calculating);
            }
        }
    }

    private bool obstaclePlaning() {
        var targetRB = Geometry.ToRangeBearing(currentTargetPosition, lastLaserReadings.LastPose);
        if (targetRB.x < TARGET_RADIUS) {
            //Reached the current target.
            Debug.Log("Planing - Reached current target.");
            lock (currentTarget) currentTarget.Pop();
            obstacles = null;
            return false;
        }
        positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, lastLaserReadings.LastPose);
        positiveTurningCenter.z = lastLaserReadings.LastPose.z;
        negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, lastLaserReadings.LastPose);
        negativeTurningCenter.z = lastLaserReadings.LastPose.z;
        var unobstructedRadius = findBothUnobstructedRadius();
        if (unobstructedRadius.x <= 0f || unobstructedRadius.y >= 0f) {
            //Reached a dead end.
            steering.Halt();
            lastLaserReadings.Write();
            throw new ArgumentException("Ra" + unobstructedRadius);
            /*Debug.Log("Planing - Reached dead end. (1) " + unobstructedRadius);
            steering.Halt();
            lock (currentTarget) {
                currentTarget.Pop();
                currentTarget.Push(TargetCommand.Backtrack);
            }
            obstacles = null;
            return false;*/
        }
        if (backwards) {
            if(Mathf.Abs(targetRB.y) < Geometry.RIGHT_ANGLE) {
                backwards = false;
                obstacles = null;
                return true;
            }
            //Try to turn around in a two/three point turn:
            if (hypothesizeTurn(unobstructedRadius)) {
                //backwards = false;
                obstacles = null;
                return true;
            }
            //Continue to go backwards:
            lastLaserReadings.LastPose.z += Geometry.HALF_CIRCLE;
            lastLaserReadings.LastPose.z %= Geometry.FULL_CIRCLE;
            targetRB.y += Geometry.HALF_CIRCLE;
            targetRB.y %= Geometry.FULL_CIRCLE;
            unobstructedRadius = findBothUnobstructedRadius();
            if (unobstructedRadius.x <= 0f || unobstructedRadius.y >= 0f) {
                //Reached a dead end.
                Debug.Log("Planing - Reached dead end. (2) " + unobstructedRadius);
                steering.Halt();
                lock (currentTarget) {
                    currentTarget.Pop();
                    currentTarget.Push(TargetCommand.Backtrack);
                }
                obstacles = null;
                return false;
            }
        }
        DrawArc(rendererX, UNOBSTRUCTED_HEIGHT, positiveTurningCenter, -Geometry.RIGHT_ANGLE + lastLaserReadings.LastPose.z, unobstructedRadius.x);
        DrawArc(rendererY, UNOBSTRUCTED_HEIGHT, negativeTurningCenter, Geometry.RIGHT_ANGLE + lastLaserReadings.LastPose.z, unobstructedRadius.y);
        Debug.Log("Target " + currentTargetPosition + ", RB" + targetRB + " Radius" + unobstructedRadius + " Pose" + lastLaserReadings.LastPose);
        if (!IsWithinFunnel(targetRB) || Mathf.Abs(targetRB.y) < MIN_CORRECTION_ANGLE) {
            //The target is not in the current reachable funnel. Move forward.
            //The robot is facing towards the target. No turn is needed.
            steering.DriveAhead(backwards);
            return true;
        } else {
            float steeringSegment = findSteeringSegment(unobstructedRadius, targetRB);
            Debug.Log("Steer " + steeringSegment);
            if(float.IsNaN(steeringSegment)) {
                steering.Halt();
                lock (currentTarget) {
                    currentTarget.Pop();
                }
                backwards = true;
                obstacles = null;
                return false;
            }
            steering.Steer(steeringSegment, backwards);
        }
        obstacles = null;
        return true;
    }

    /*
     * Tries to turn the vehicle around. Input is an unobstructedRadius for left and right hand turns.
     * The lastLaserReadings are used as an input as well.
     * Returns true if a turn is possible and will be executed,
     *         false otherwise.
     */
    private bool hypothesizeTurn(Vector2 unobstructedRadius) {
        if (unobstructedRadius.x >= Geometry.RIGHT_ANGLE) {
            if (unobstructedRadius.x >= Geometry.HALF_CIRCLE) {
                Debug.Log("Planing - Positive full turning.");
                steering.SteerForward(Geometry.HALF_CIRCLE);
                return true;
            }
            Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, Geometry.EIGHTH_CIRCLE, lastLaserReadings.LastPose);
            movedPose.z = (lastLaserReadings.LastPose.z - Geometry.RIGHT_ANGLE) % Geometry.FULL_CIRCLE;
            if (findPositiveUnobstructedRadius(movedPose) >= Geometry.RIGHT_ANGLE) {
                Debug.Log("Planing - Positive half turning.");
                steering.SteerForward(Geometry.RIGHT_ANGLE);
                //lock (currentTarget) currentTarget.Push(TargetCommand.Turn);
                return true;
            }
        }
        if (unobstructedRadius.y <= -Geometry.RIGHT_ANGLE) {
            if (unobstructedRadius.y <= -Geometry.HALF_CIRCLE) {
                Debug.Log("Planing - Negative full turning.");
                steering.SteerForward(-Geometry.HALF_CIRCLE);
                return true;
            }
            Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, -Geometry.EIGHTH_CIRCLE, lastLaserReadings.LastPose);
            if (findNegativeUnobstructedRadius(movedPose) >= Geometry.RIGHT_ANGLE) {
                Debug.Log("Planing - Negative half turning.");
                steering.SteerForward(-Geometry.RIGHT_ANGLE);
                //lock (currentTarget) currentTarget.Push(TargetCommand.Turn);
                return true;
            }
        }
        //TODO: If the above failed, we may be able to turn around by reversing first. This would mean code changes to the TargetCommand.Turn as well.
        // Skip that for now for better performance in narrow situations.
        //TODO: If the above succeds we might get stuck because the turn does not check what happens after the turn.
        return false;
    }

    private Vector2 findBothUnobstructedRadius() {
        obstacles = new List<Vector3>();
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        Vector2 unobstructedRadius = new Vector2(ALPHA, ALPHA);
        for (int i = 0; i < lastLaserReadings.ReadingsCount; i++) {
            if (Mathf.Abs(lastLaserReadings.ReadingsRB[i].y) > BETA) continue;
            if (IsWithinFunnel(lastLaserReadings.ReadingsRB[i])) {
                if (lastLaserReadings.ReadingsRB[i].x < MIN_OBSTACLE_DISTANCE) {
                    //The obstacle is too close.
                    //We will not be able to steer around the obstacle. This means we were unable to steer around the feature or the set position is unreachable from our current position.
                    return Vector2.zero;
                }
                //The obstacle is in front of our possible movements.
                obstacles.Add(lastLaserReadings.Readings[i]);
                if (lastLaserReadings.ReadingsRB[i].y >= 0f) {
                    var turningRB = Geometry.ToRangeBearing2(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        turningRB.y = Geometry.RIGHT_ANGLE + turningRB.y;
                        if (turningRB.y < unobstructedRadius.x) unobstructedRadius.x = turningRB.y;
                    }
                } else {
                    var turningRB = Geometry.ToRangeBearing2(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        turningRB.y = Geometry.RIGHT_ANGLE - turningRB.y;
                        if (turningRB.y < unobstructedRadius.y) unobstructedRadius.y = turningRB.y;
                    }
                }
            }
        }
        unobstructedRadius.x -= UNOBSTRUCTED_OFFSET;
        unobstructedRadius.y = UNOBSTRUCTED_OFFSET - unobstructedRadius.y;
        return unobstructedRadius;
    }

    private float findNegativeUnobstructedRadius(Vector3 origin) {
        var negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, origin);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        float unobstructedRadius = Geometry.HALF_CIRCLE;
        for (int i = 0; i < lastLaserReadings.ReadingsCount; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], origin);
            if (Mathf.Abs(rangeBearing.y) > Geometry.HALF_CIRCLE) continue;
            if (IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.y <= 0f) {
                    var turningRB = Geometry.ToRangeBearing2(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Mathf.Asin(rangeBearing.x * Mathf.Cos(rangeBearing.y) / turningRB.x);
                        currentAngle = (currentAngle + Geometry.FULL_CIRCLE) % Geometry.FULL_CIRCLE;
                        if (currentAngle < unobstructedRadius) unobstructedRadius = currentAngle;
                    }
                    throw new NotImplementedException("turningRb can not be calculated through negativeTurningCenter!");
                }
            }
        }
        return UNOBSTRUCTED_OFFSET - unobstructedRadius;
    }

    private float findPositiveUnobstructedRadius(Vector3 origin) {
        var positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, origin);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        float unobstructedRadius = Geometry.HALF_CIRCLE;
        for (int i = 0; i < lastLaserReadings.ReadingsCount; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], origin);
            if (Mathf.Abs(rangeBearing.y) > Geometry.HALF_CIRCLE) continue;
            if (IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.y >= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Mathf.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Mathf.Asin(rangeBearing.x * Mathf.Cos(rangeBearing.y) / distance);
                        currentAngle = (currentAngle + Geometry.FULL_CIRCLE) % Geometry.FULL_CIRCLE;
                        if (currentAngle < unobstructedRadius) unobstructedRadius = currentAngle;
                    }
                    throw new NotImplementedException("turningRb can not be calculated through positiveTurningCenter!");
                }
            }
        }
        return unobstructedRadius - UNOBSTRUCTED_OFFSET;
    }

    private float findSteeringSegment(Vector2 unobstructedRadius, Vector2 targetRB) {
        //Calculate the segment of circle that the robot has to turn at max steering angle: 
        var bearing = Geometry.RIGHT_ANGLE - Mathf.Abs(targetRB.y);
        var c = MainMenu.Physics.turningRadiusSquared + targetRB.x * targetRB.x - 2f * MainMenu.Physics.turningRadius * targetRB.x * Mathf.Cos(bearing);
        float steeringSegment = (Mathf.Asin((targetRB.x * Mathf.Sin(bearing)) / Mathf.Sqrt(c)) - Mathf.Asin(Mathf.Sqrt(c - MainMenu.Physics.turningRadiusSquared) / Mathf.Sqrt(c)));
        //float lineLength;
        //if(steeringSegment > Geometry.RIGHT_ANGLE) lineLength = targetRB.x - MainMenu.Physics.turningRadius;
        //else lineLength = targetRB.x - MainMenu.Physics.turningRadius * steeringSegment;
        if (bearing < 0) steeringSegment = -steeringSegment;
        //Find the closest steerable way towards the currentTargetPosition
        //TODO: A steering plan consists of two elements: A line and a turn.
        if (steeringSegment > unobstructedRadius.x) steeringSegment = unobstructedRadius.x;
        else if (steeringSegment < unobstructedRadius.y) steeringSegment = unobstructedRadius.y;
        else {
            float f = 0f;
            for (; f < MAX_OFFSET_ANGLE; f += OBSTACLE_PLANING_STEP) {
                var currentAngle = steeringSegment + f;
                if (currentAngle <= unobstructedRadius.x) {
                    if (currentAngle >= unobstructedRadius.y) {
                        Vector3 line = Geometry.Rotate(lastLaserReadings.LastPose, currentAngle >= 0f ? positiveTurningCenter : negativeTurningCenter, currentAngle);
                        line.z = lastLaserReadings.LastPose.z + currentAngle;
                        //targetRB.x is roughly the real target distance from line plus not more than MainMenu.Physics.turningRadius.
                        if (checkLine(line, 0.1f)) break;
                    } else {
                        f = unobstructedRadius.y - steeringSegment - OBSTACLE_PLANING_STEP;
                    }
                } else {
                    f = MAX_OFFSET_ANGLE;
                    break;
                }
            }
            float g = OBSTACLE_PLANING_STEP;
            for (; g < f; g += OBSTACLE_PLANING_STEP) {
                var currentAngle = steeringSegment - g;
                if (currentAngle >= unobstructedRadius.y) {
                    if (currentAngle <= unobstructedRadius.x) {
                        Vector3 line = Geometry.Rotate(lastLaserReadings.LastPose, currentAngle >= 0f ? positiveTurningCenter : negativeTurningCenter, currentAngle);
                        line.z = lastLaserReadings.LastPose.z + currentAngle;
                        if (checkLine(line, 0.1f)) break;
                    } else {
                        g = steeringSegment - unobstructedRadius.x - OBSTACLE_PLANING_STEP;
                    }
                } else {
                    g = MAX_OFFSET_ANGLE;
                    break;
                }
            }
            if (f >= MAX_OFFSET_ANGLE && g >= MAX_OFFSET_ANGLE) {
                //This means that either the steerable range was smaller than OBSTACLE_PLANING_STEP
                //or the target is blocked by an obstacle -> remove edge from graph
                globalGraph.DisconnectNode(currentTargetPosition);
                Debug.Log("Disconnecting " + currentTargetPosition);
                return float.NaN;
            }
            steeringSegment += (f < g ? f : g);
        }
        if(steeringSegment > 0) DrawArc(rendererSteering, STEERING_HEIGHT, positiveTurningCenter, -Geometry.RIGHT_ANGLE + lastLaserReadings.LastPose.z, steeringSegment);
        else DrawArc(rendererSteering, STEERING_HEIGHT, negativeTurningCenter, Geometry.RIGHT_ANGLE + lastLaserReadings.LastPose.z, steeringSegment);
        return steeringSegment;
    }

    private bool checkLine(Vector3 line, float length) {
        foreach (Vector3 obstacle in obstacles) {
            Vector2 obstacleRB = Geometry.ToRangeBearing(obstacle, line);
            if (Mathf.Abs(Mathf.Sin(obstacleRB.y) * obstacleRB.x) < MIN_OBSTACLE_DISTANCE && Mathf.Abs(Mathf.Cos(obstacleRB.y) * obstacleRB.x) < length) return false;
        }
        return true;
    }

    //Checks wether the feature, provided as range and bearing, is within the currently drivable funnel of the robot
    public static bool IsWithinFunnel(Vector2 featureRB) {
        if (featureRB.y == 0f) return true;
        return Mathf.Sin(Mathf.Abs(featureRB.y)) * MainMenu.Physics.turningDiameter - MainMenu.Physics.halfWheelbase < featureRB.x;
        //This is not completly correct. The turning circle should only be moved half wheelbase to the side!
    }

    public static void DrawArc(LineRenderer renderer, float height, Vector2 center, float offset, float angle) {
        var arcPoints = new List<Vector3>();
        float sign = Mathf.Sign(angle);
        angle *= sign;
        for(float f = 0; f < angle; f += ARC_STEP) {
            float x = Mathf.Cos(sign * f + offset) * MainMenu.Physics.turningRadius;
            float y = Mathf.Sin(sign * f + offset) * MainMenu.Physics.turningRadius;
            arcPoints.Add(new Vector3(x+center.x,height,y+center.y));
        }
        renderer.positionCount = arcPoints.Count;
        renderer.SetPositions(arcPoints.ToArray());
    }
}
}