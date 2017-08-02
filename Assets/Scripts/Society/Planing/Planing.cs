using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

enum TargetCommand {
    RandomMove,
    ExplorePosition,
    Backtrack,
    Turn
}

class PlaningInputData {

    public Vector3 LastPose;
    public Vector3[] Readings;
    public bool[] Invalid;

    public PlaningInputData() { }

    public PlaningInputData(Vector3[] readings, bool[] invalid) {
        Readings = new Vector3[readings.Length];
        for (int i = 0; i < Readings.Length; i++) Readings[i] = readings[i];
    }
}

class Planing: MonoBehaviour {

    public const int GRAPH_FEED_INTERVAL = 5;
    public const float HALF_CIRCLE = (float) Math.PI;
    public const float RIGHT_ANGLE = HALF_CIRCLE / 2f;
    public const float FULL_CIRCLE = HALF_CIRCLE * 2f;
    public const float EIGHTH_CIRCLE = RIGHT_ANGLE / 2f;
    //Parameters:
    public const float ALPHA = HALF_CIRCLE + HALF_CIRCLE / 4f;
    public const float OBSTACLE_PLANING_STEP = HALF_CIRCLE / 36f;
    public const float MIN_OBSTACLE_DISTANCE = 0.3f;
    public const float UNOBSTRUCTED_OBSTACLE_MULTIPLIER = 1.5f;
    public const float TARGET_RADIUS = 0.1f;
    public const float MAX_OFFSET_ANGLE = HALF_CIRCLE;
    public const float MIN_CORRECTION_ANGLE = HALF_CIRCLE / 180f;
    //Calculated once at Startup:
    public static float UNOBSTRUCTED_OFFSET;

    public static Planing singleton;
    /***********************************************************************
    public object LocalMapLock = new object();
    public object GlobalMapLock = new object();
    public object ServerMapLock = new object();

    public LocalClientMap LocalMap;
    public GlobalClientMap GlobalMap;
    public ServerMapMessage ServerMap;
    ***********************************************************************/
    
    public PlaningInputData LaserReadings {
        private get { lock (laserReadingsLock) return currentLaserReadings; }
        set { lock (laserReadingsLock) currentLaserReadings = value; }
    }
    public Graph GlobalGraph = new Graph();

    private CarDrive steering;
    private PositionHistory positionHistory;
    private Stack<TargetCommand> currentTarget = new Stack<TargetCommand>();
    private Vector2 currentTargetPosition = new Vector2(1f, 0f);
    private Queue<Vector2> currentPath;
    private object laserReadingsLock = new object();
    private PlaningInputData currentLaserReadings = null;
    private PlaningInputData lastLaserReadings = null;
    private int graphCounter = 0;
    private bool backwards = false;
    private Vector2 positiveTurningCenter;
    private Vector2 negativeTurningCenter;
    private List<Vector3> obstacles;

    public void Awake() {
        singleton = this;
        currentTarget.Push(TargetCommand.RandomMove);
        steering = gameObject.GetComponent<CarDrive>();
        positionHistory = gameObject.GetComponent<PositionHistory>();
        UNOBSTRUCTED_OFFSET =  2f * (float) Math.Asin(MIN_OBSTACLE_DISTANCE * UNOBSTRUCTED_OBSTACLE_MULTIPLIER / Math.Sqrt(UNOBSTRUCTED_OBSTACLE_MULTIPLIER * 2f * MainMenu.Physics.turningRadius * MIN_OBSTACLE_DISTANCE));
        StartCoroutine("workerRoutine");
    }

    public TargetCommand GetCurrentTarget() {
        lock(currentTarget) return currentTarget.Peek();
    }

    public Vector2 GetCurrentTargetPosition() {
        return currentTargetPosition;
    }

    private IEnumerator workerRoutine() {
        yield return new WaitWhile(() => lastLaserReadings == currentLaserReadings);
        lastLaserReadings = LaserReadings;
        GlobalGraph.Feed(lastLaserReadings);
        defineNewTarget();
        bool wasUsed = false;
        while (true) {
            if(wasUsed) yield return new WaitWhile(() => lastLaserReadings == currentLaserReadings);
            lastLaserReadings = LaserReadings;
            wasUsed = false;
            if (currentTarget.Peek() == TargetCommand.ExplorePosition) {
                if (obstaclePlaning()) {
                    wasUsed = true;
                    graphCounter++;
                    graphCounter %= 5;
                    if (graphCounter == 0) GlobalGraph.Feed(lastLaserReadings);
                }
            } else if (currentTarget.Peek() == TargetCommand.Turn) {
                //Wait for the turn to finish:
                yield return new WaitWhile(() => steering.IsTurning());
                wasUsed = true;
                currentTarget.Pop();
            } else defineNewTarget();

        }
    }

    private bool obstaclePlaning() {
        PositionData pos = positionHistory.GetNewestThreadSafe();
        lastLaserReadings.LastPose = new Vector3(pos.position.x, pos.position.z, pos.heading / 180f * HALF_CIRCLE);
        var targetRB = Geometry.ToRangeBearing(currentTargetPosition, lastLaserReadings.LastPose);
        if (targetRB.x < TARGET_RADIUS) {
            //Reached the current target.
            currentTarget.Pop();
            defineNewTarget();
            return false;
        }
        if (!Geometry.IsWithinFunnel(targetRB)) {
            //TODO! the target is not in the current reachable funnel. Do something
            throw new NotImplementedException();
        }
        positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, RIGHT_ANGLE, lastLaserReadings.LastPose);
        negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -RIGHT_ANGLE, lastLaserReadings.LastPose);
        var unobstructedRadius = findBothUnobstructedRadius(out obstacles);
        if (unobstructedRadius.x <= 0f || unobstructedRadius.y >= 0f) {
            //Reached a dead end.
            steering.Halt();
            GlobalGraph.ReachedDeadEnd(lastLaserReadings.LastPose);
            currentTarget.Pop();
            currentTarget.Push(TargetCommand.Backtrack);
            return false;
        }
        if (backwards) {
            //Try to turn around in a two/three point turn:
            if (hypothesizeTurn(unobstructedRadius)) {
                backwards = false;
                return true;
            }
            //Continue to go backwards:
            lastLaserReadings.LastPose.z += HALF_CIRCLE;
            lastLaserReadings.LastPose.z %= FULL_CIRCLE;
            targetRB.y += HALF_CIRCLE;
            targetRB.y %= FULL_CIRCLE;
            unobstructedRadius = findBothUnobstructedRadius(out obstacles);
        }
        if (Math.Abs(targetRB.y) < MIN_CORRECTION_ANGLE) {
            //The robot is facing towards the target. No turn is needed.
            targetRB.y = 0.0f;
        }
        float steeringSegment = findSteeringSegment(unobstructedRadius, targetRB);
        obstacles = null;
        steering.Steer(steeringSegment, backwards);
        return true;
    }

    /*
     * Tries to turn the vehicle around. Input is an unobstructedRadius for left and right hand turns.
     * The lastLaserReadings are used as an input as well.
     * Returns true if a turn is possible and will be executed,
     *         false otherwise.
     */
    private bool hypothesizeTurn(Vector2 unobstructedRadius) {
        if (unobstructedRadius.x >= RIGHT_ANGLE) {
            if (unobstructedRadius.x >= HALF_CIRCLE) {
                steering.SteerForward(HALF_CIRCLE);
                return true;
            }
            Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, EIGHTH_CIRCLE, lastLaserReadings.LastPose);
            movedPose.z = (lastLaserReadings.LastPose.z - RIGHT_ANGLE) % FULL_CIRCLE;
            if (findPositiveUnobstructedRadius(movedPose) >= RIGHT_ANGLE) {
                steering.SteerForward(RIGHT_ANGLE);
                currentTarget.Push(TargetCommand.Turn);
                return true;
            }
        }
        if (unobstructedRadius.y <= -RIGHT_ANGLE) {
            if (unobstructedRadius.y <= -HALF_CIRCLE) {
                steering.SteerForward(-HALF_CIRCLE);
                return true;
            }
            Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, -EIGHTH_CIRCLE, lastLaserReadings.LastPose);
            if (findNegativeUnobstructedRadius(movedPose) >= RIGHT_ANGLE) {
                steering.SteerForward(-RIGHT_ANGLE);
                currentTarget.Push(TargetCommand.Turn);
                return true;
            }
        }
        //TODO: If the above failed, we may be able to turn around by reversing first. This would mean code changes to the TargetCommand.Turn as well.
        // Skip that for now for better performance in narrow situations.
        //TODO: If the above succedes we might get stuck because the turn does not check what happens after the turn.
        return false;
    }

    private Vector2 findBothUnobstructedRadius(out List<Vector3> obstacles) {
        obstacles = new List<Vector3>();
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        Vector2 unobstructedRadius = new Vector2(ALPHA, -ALPHA);
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], lastLaserReadings.LastPose);
            Debug.Log("i " + i + ", r " + rangeBearing.x + ", b " + rangeBearing.y);//TODO: index is bearing in degree!?
            if (Math.Abs(rangeBearing.y) > ALPHA) continue;
            if (Geometry.IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.x < MIN_OBSTACLE_DISTANCE) {
                    //The obstacle is too close.
                    //We will not be able to steer around the obstacle. This means we were unable to steer around the feature or the set position is unreachable from our current position.
                    return Vector2.zero;
                }
                obstacles.Add(lastLaserReadings.Readings[i]);
                if (rangeBearing.y >= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Math.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = (float)Math.Atan2(lastLaserReadings.Readings[i].x - positiveTurningCenter.x, lastLaserReadings.Readings[i].z - positiveTurningCenter.y);
                        if (currentAngle < unobstructedRadius.x) unobstructedRadius.x = currentAngle;
                    }
                } else {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Math.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = (float)Math.Atan2(lastLaserReadings.Readings[i].x - negativeTurningCenter.x, lastLaserReadings.Readings[i].z - negativeTurningCenter.y);
                        if (currentAngle > unobstructedRadius.y) {
                            unobstructedRadius.y = currentAngle;
                        }
                    }
                }
            }
        }
        unobstructedRadius.x -= UNOBSTRUCTED_OFFSET;
        unobstructedRadius.y = UNOBSTRUCTED_OFFSET - unobstructedRadius.y;
        return unobstructedRadius;
    }

    private float findNegativeUnobstructedRadius(Vector3 origin) {
        var negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -RIGHT_ANGLE, origin);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        float unobstructedRadius = -HALF_CIRCLE;
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], origin);
            Debug.Log("i " + i + ", r " + rangeBearing.x + ", b " + rangeBearing.y);//TODO: index is bearing in degree!?
            if (Math.Abs(rangeBearing.y) > HALF_CIRCLE) continue;
            if (Geometry.IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.y <= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Math.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = (float)Math.Atan2(lastLaserReadings.Readings[i].x - negativeTurningCenter.x, lastLaserReadings.Readings[i].z - negativeTurningCenter.y);
                        if (currentAngle > unobstructedRadius) {
                            unobstructedRadius = currentAngle;
                        }
                    }
                }
            }
        }
        return UNOBSTRUCTED_OFFSET - unobstructedRadius;
    }

    private float findPositiveUnobstructedRadius(Vector3 origin) {
        var positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, RIGHT_ANGLE, origin);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        float unobstructedRadius = -HALF_CIRCLE;
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], origin);
            Debug.Log("i " + i + ", r " + rangeBearing.x + ", b " + rangeBearing.y);//TODO: index is bearing in degree!?
            if (Math.Abs(rangeBearing.y) > HALF_CIRCLE) continue;
            if (Geometry.IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.y >= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Math.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = (float)Math.Atan2(lastLaserReadings.Readings[i].x - positiveTurningCenter.x, lastLaserReadings.Readings[i].z - positiveTurningCenter.y);
                        if (currentAngle < unobstructedRadius) unobstructedRadius = currentAngle;
                    }
                }
            }
        }
        return unobstructedRadius - UNOBSTRUCTED_OFFSET;
    }

    private float findSteeringSegment(Vector2 unobstructedRadius, Vector2 targetRB) {
        //Calculate the segment of circle that the robot has to turn at max steering angle: 
        var bearing = RIGHT_ANGLE - Math.Abs(targetRB.y);
        var c = MainMenu.Physics.turningRadiusSquared + targetRB.x * targetRB.x - 2f * MainMenu.Physics.turningRadius * targetRB.x * Math.Cos(bearing);
        float steeringSegment = (float)(Math.Asin((targetRB.x * Math.Sin(bearing)) / Math.Sqrt(c)) - Math.Asin(Math.Sqrt(c - MainMenu.Physics.turningRadiusSquared) / Math.Sqrt(c)));
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
                        if (checkLine(line, targetRB.x)) break;
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
                        if (checkLine(line, targetRB.x)) break;
                    }
                } else {
                    g = MAX_OFFSET_ANGLE;
                    break;
                }
            }
            if (f >= MAX_OFFSET_ANGLE && g >= MAX_OFFSET_ANGLE) {
                //TODO! something went wrong. This would mean that the step was too big so the unobstructed range was missed.
                throw new NotImplementedException();
            }
            steeringSegment += (f < g ? f : g);
        }
        return steeringSegment;
    }

    private bool checkLine(Vector3 line, float length) {
        foreach(Vector3 obstacle in obstacles) {
            Vector2 obstacleRB = Geometry.ToRangeBearing(obstacle, line);
            if (Math.Sin(obstacleRB.y) * obstacleRB.x < MIN_OBSTACLE_DISTANCE && Math.Cos(obstacleRB.y) * obstacleRB.x < length) return false;
        }
        return true;
    }

    private void defineNewTarget() {
        if (currentTarget.Peek() == TargetCommand.RandomMove) {
            currentTargetPosition = GlobalGraph.GetNewTarget();
            currentTarget.Push(TargetCommand.ExplorePosition);
            return;
        }
        if (currentTarget.Peek() == TargetCommand.ExplorePosition) {
            currentTargetPosition = currentPath.Dequeue();
            if(currentTargetPosition == null) {
                currentPath = null;
                currentTarget.Pop();
                defineNewTarget();
            }
            return;
        }
        if (currentTarget.Peek() == TargetCommand.Backtrack) {
            currentTarget.Pop();
            currentPath = GlobalGraph.GetUnexploredNodePath();
            currentTarget.Push(TargetCommand.ExplorePosition);
            currentTargetPosition = currentPath.Dequeue();
            backwards = !backwards;
            return;
        }
        //TODO: currentTarget is null.
        throw new NotImplementedException();
    }
}
