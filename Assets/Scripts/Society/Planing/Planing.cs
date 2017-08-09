﻿using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum TargetCommand {
    RandomMove,
    ExplorePosition,
    Backtrack,
    Turn
}

public class PlaningInputData {
    
    public Vector3 LastPose;
    public Vector3[] Readings;
    public Vector2[] ReadingsRB;

    public PlaningInputData() { }

    public PlaningInputData(Vector3[] readings, bool[] invalid, int invalidCount) {
        Readings = new Vector3[readings.Length - invalidCount];
        ReadingsRB = new Vector2[readings.Length - invalidCount];
        int count = 0;
        for (int i = 0; i < readings.Length; i++) {
            if (!invalid[i]) Readings[count++] = readings[i];
        }
    }
}

public class Planing: MonoBehaviour {

    //Parameters:
    public const int GRAPH_FEED_INTERVAL = 5;
    public const float ALPHA = Geometry.HALF_CIRCLE + Geometry.HALF_CIRCLE / 4f;
    public const float OBSTACLE_PLANING_STEP = Geometry.HALF_CIRCLE / 36f;
    public const float MIN_OBSTACLE_DISTANCE = 0.3f;
    public const float UNOBSTRUCTED_OBSTACLE_MULTIPLIER = 1.5f;
    public const float TARGET_RADIUS = 0.1f;
    public const float MAX_OFFSET_ANGLE = Geometry.HALF_CIRCLE;
    public const float MIN_CORRECTION_ANGLE = Geometry.HALF_CIRCLE / 180f;
    //Calculated once at Startup:
    public static float UNOBSTRUCTED_OFFSET;

    public static Planing singleton;
    
    public PlaningInputData LaserReadings {
        private get { lock (laserReadingsLock) return currentLaserReadings; }
        set { lock (laserReadingsLock) currentLaserReadings = value; }
    }
    public Graph GlobalGraph;

    private CarDrive steering;
    private PositionHistory positionHistory;
    private Stack<TargetCommand> currentTarget = new Stack<TargetCommand>();
    private Vector2 currentTargetPosition = new Vector2(1f, 0f);
    private LinkedList<Vector2> currentPath;
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
        UNOBSTRUCTED_OFFSET = Mathf.Acos(1f - UNOBSTRUCTED_OBSTACLE_MULTIPLIER * MIN_OBSTACLE_DISTANCE / MainMenu.Physics.turningRadius);
        GlobalGraph = gameObject.GetComponent<Graph>();
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
                yield return new WaitWhile(steering.IsTurning);
                wasUsed = true;
                currentTarget.Pop();
            } else defineNewTarget();

        }
    }

    private bool obstaclePlaning() {
        PositionData pos = positionHistory.GetNewestThreadSafe();
        lastLaserReadings.LastPose = new Vector3(pos.position.x, pos.position.z, pos.heading * Mathf.PI / 180f);
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) lastLaserReadings.ReadingsRB[i] = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], lastLaserReadings.LastPose);
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
        positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, lastLaserReadings.LastPose);
        negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, lastLaserReadings.LastPose);
        var unobstructedRadius = findBothUnobstructedRadius(out obstacles);
        if (unobstructedRadius.x <= 0f || unobstructedRadius.y >= 0f) {
            //Reached a dead end.
            steering.Halt();
            //GlobalGraph.ReachedDeadEnd(lastLaserReadings.LastPose);
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
            lastLaserReadings.LastPose.z += Geometry.HALF_CIRCLE;
            lastLaserReadings.LastPose.z %= Geometry.FULL_CIRCLE;
            targetRB.y += Geometry.HALF_CIRCLE;
            targetRB.y %= Geometry.FULL_CIRCLE;
            unobstructedRadius = findBothUnobstructedRadius(out obstacles);
        }
        if (Mathf.Abs(targetRB.y) < MIN_CORRECTION_ANGLE) {
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
        if (unobstructedRadius.x >= Geometry.RIGHT_ANGLE) {
            if (unobstructedRadius.x >= Geometry.HALF_CIRCLE) {
                steering.SteerForward(Geometry.HALF_CIRCLE);
                return true;
            }
            Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, Geometry.EIGHTH_CIRCLE, lastLaserReadings.LastPose);
            movedPose.z = (lastLaserReadings.LastPose.z - Geometry.RIGHT_ANGLE) % Geometry.FULL_CIRCLE;
            if (findPositiveUnobstructedRadius(movedPose) >= Geometry.RIGHT_ANGLE) {
                steering.SteerForward(Geometry.RIGHT_ANGLE);
                currentTarget.Push(TargetCommand.Turn);
                return true;
            }
        }
        if (unobstructedRadius.y <= -Geometry.RIGHT_ANGLE) {
            if (unobstructedRadius.y <= -Geometry.HALF_CIRCLE) {
                steering.SteerForward(-Geometry.HALF_CIRCLE);
                return true;
            }
            Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, -Geometry.EIGHTH_CIRCLE, lastLaserReadings.LastPose);
            if (findNegativeUnobstructedRadius(movedPose) >= Geometry.RIGHT_ANGLE) {
                steering.SteerForward(-Geometry.RIGHT_ANGLE);
                currentTarget.Push(TargetCommand.Turn);
                return true;
            }
        }
        //TODO: If the above failed, we may be able to turn around by reversing first. This would mean code changes to the TargetCommand.Turn as well.
        // Skip that for now for better performance in narrow situations.
        //TODO: If the above succeds we might get stuck because the turn does not check what happens after the turn.
        return false;
    }

    private Vector2 findBothUnobstructedRadius(out List<Vector3> obstacles) {
        obstacles = new List<Vector3>();
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        Vector2 unobstructedRadius = new Vector2(ALPHA, -ALPHA);
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            if (Mathf.Abs(lastLaserReadings.ReadingsRB[i].y) > ALPHA) continue;
            if (Geometry.IsWithinFunnel(lastLaserReadings.ReadingsRB[i])) {
                //The obstacle is in front of our possible movements.
                if (lastLaserReadings.ReadingsRB[i].x < MIN_OBSTACLE_DISTANCE) {
                    //The obstacle is too close.
                    //We will not be able to steer around the obstacle. This means we were unable to steer around the feature or the set position is unreachable from our current position.
                    return Vector2.zero;
                }
                obstacles.Add(lastLaserReadings.Readings[i]);
                if (lastLaserReadings.ReadingsRB[i].y >= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Mathf.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Mathf.Atan2(lastLaserReadings.Readings[i].x - positiveTurningCenter.x, lastLaserReadings.Readings[i].z - positiveTurningCenter.y);
                        if (currentAngle < unobstructedRadius.x) unobstructedRadius.x = currentAngle;
                    }
                } else {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Mathf.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Mathf.Atan2(lastLaserReadings.Readings[i].x - negativeTurningCenter.x, lastLaserReadings.Readings[i].z - negativeTurningCenter.y);
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
        var negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, origin);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        float unobstructedRadius = -Geometry.HALF_CIRCLE;
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], origin);
            Debug.Log("i " + i + ", r " + rangeBearing.x + ", b " + rangeBearing.y);//TODO: index is bearing in degree!?
            if (Mathf.Abs(rangeBearing.y) > Geometry.HALF_CIRCLE) continue;
            if (Geometry.IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.y <= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Mathf.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Mathf.Atan2(lastLaserReadings.Readings[i].x - negativeTurningCenter.x, lastLaserReadings.Readings[i].z - negativeTurningCenter.y);
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
        var positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, origin);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        float unobstructedRadius = -Geometry.HALF_CIRCLE;
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], origin);
            Debug.Log("i " + i + ", r " + rangeBearing.x + ", b " + rangeBearing.y);//TODO: index is bearing in degree!?
            if (Mathf.Abs(rangeBearing.y) > Geometry.HALF_CIRCLE) continue;
            if (Geometry.IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                if (rangeBearing.y >= 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Mathf.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Mathf.Atan2(lastLaserReadings.Readings[i].x - positiveTurningCenter.x, lastLaserReadings.Readings[i].z - positiveTurningCenter.y);
                        if (currentAngle < unobstructedRadius) unobstructedRadius = currentAngle;
                    }
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
                        if (checkLine(line, targetRB.x)) break;
                    } else {
                        g = steeringSegment - unobstructedRadius.x - OBSTACLE_PLANING_STEP;
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
            if (Mathf.Sin(obstacleRB.y) * obstacleRB.x < MIN_OBSTACLE_DISTANCE && Mathf.Cos(obstacleRB.y) * obstacleRB.x < length) return false;
        }
        return true;
    }

    private void defineNewTarget() {
        if (currentTarget.Peek() == TargetCommand.RandomMove) {
            if(!GlobalGraph.GetNewTarget(out currentTargetPosition)) {
                steering.Halt();
                //Backtrack as we don't have any target left at the moment.
                currentPath = GlobalGraph.GetUnexploredNodePath(lastLaserReadings.LastPose);
                currentTargetPosition = currentPath.First.Value;
                currentPath.RemoveFirst();
                backwards = !backwards;
            }
            currentTarget.Push(TargetCommand.ExplorePosition);
            return;
        }
        if (currentTarget.Peek() == TargetCommand.ExplorePosition) {
            if (currentPath.Count <= 0) {
                currentPath = null;
                currentTarget.Pop();
                defineNewTarget();
            } else {
                currentTargetPosition = currentPath.First.Value;
                currentPath.RemoveFirst();
            }
            return;
        }
        if (currentTarget.Peek() == TargetCommand.Backtrack) {
            currentTarget.Pop();
            currentPath = GlobalGraph.GetUnexploredNodePath(lastLaserReadings.LastPose);
            currentTarget.Push(TargetCommand.ExplorePosition);
            currentTargetPosition = currentPath.First.Value;
            currentPath.RemoveFirst();
            backwards = !backwards;
            return;
        }
        //TODO: currentTarget is null.
        throw new NotImplementedException();
    }
}
