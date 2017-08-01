using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

enum TargetCommand {
    RandomMove,
    Turn,
    ExplorePosition,
    Backtrack
    //TODO:More Commands?
}

class PlaningInputData {

    public Vector3 FirstPose;
    public Vector3 LastPose;
    public Vector3[] Readings;
    public bool[] Invalid;

    public PlaningInputData() { }

    public PlaningInputData(PositionData firstPos, PositionData lastPos, Vector3[] readings, bool[] invalid) {
        FirstPose = new Vector3(firstPos.position.x, firstPos.position.z, (float)(firstPos.heading / 180f * Math.PI));
        LastPose = new Vector3(lastPos.position.x, lastPos.position.z, (float)(lastPos.heading / 180f * Math.PI));
        Readings = new Vector3[readings.Length];
        for (int i = 0; i < Readings.Length; i++) Readings[i] = readings[i];
    }
}

class Planing: MonoBehaviour {

    public const int GRAPH_FEED_INTERVAL = 5;
    public const float RIGHT_ANGLE = (float) (Math.PI / 2f);
    public const float FULL_CIRCLE = (float)(Math.PI * 2f);
    //Parameters:
    public const float ALPHA = (float)(RIGHT_ANGLE + Math.PI / 4f);
    public const float OBSTACLE_PLANING_STEP = (float)(Math.PI / 180);
    public const float MIN_OBSTACLE_DISTANCE = 0.3f;
    public const float TARGET_RADIUS = 0.1f;
    public const float MAX_OFFSET_ANGLE = RIGHT_ANGLE;

    //Calculated once at Startup:
    public float UNOBSTRUCTED_OFFSET;

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
        get { lock (laserReadingsLock) return currentLaserReadings; }
        set { lock (laserReadingsLock) currentLaserReadings = value; }
    }
    public Graph GlobalGraph = new Graph();

    private Stack<TargetCommand> currentTarget = new Stack<TargetCommand>();
    private Vector2 currentTargetPosition = new Vector2(1f, 0f);
    private object laserReadingsLock = new object();
    private PlaningInputData currentLaserReadings = null;
    private PlaningInputData lastLaserReadings = null;
    private int graphCounter = 4;
    private CarDrive steering;

    public void Awake() {
        singleton = this;
        currentTarget.Push(TargetCommand.RandomMove);
        steering = gameObject.GetComponent<CarDrive>();
        UNOBSTRUCTED_OFFSET =  2f * (float) Math.Asin(MIN_OBSTACLE_DISTANCE * 2f / Math.Sqrt(4f * MainMenu.Physics.turningRadius * MIN_OBSTACLE_DISTANCE));
        StartCoroutine("workerRoutine");
    }

    public TargetCommand GetCurrentTarget() {
        lock(currentTarget) return currentTarget.Peek();
    }

    public Vector2 GetCurrentTargetPosition() {
        return currentTargetPosition;
    }

    private IEnumerator workerRoutine() {
        while (true) {
            yield return new WaitWhile(() => lastLaserReadings == currentLaserReadings);
            lastLaserReadings = LaserReadings;
            if (currentTarget.Peek() == TargetCommand.ExplorePosition) obstaclePlaning();
            else {
                //TODO!
            }
            graphCounter++;
            graphCounter %= 5;
            if (graphCounter == 0) GlobalGraph.Feed(lastLaserReadings);
        }
    }

    private void obstaclePlaning() {
        var targetRB = Geometry.ToRangeBearing(currentTargetPosition, lastLaserReadings.LastPose);
        if (targetRB.x < TARGET_RADIUS) {
            currentTarget.Pop();
            //TODO!
        }
        if (!Geometry.IsWithinFunnel(targetRB)) {
            //TODO! the target is not in the current reachable funnel. Do something 
        }
        //TODO: Reaching dead end should let the obstacle detection run different.
        bool movingBackwards = false;
        if(Math.Abs(Geometry.ToRangeBearing(lastLaserReadings.FirstPose, lastLaserReadings.LastPose).y) > RIGHT_ANGLE) {
            movingBackwards = true;
            lastLaserReadings.LastPose.z += (float) Math.PI;
            lastLaserReadings.LastPose.z %= FULL_CIRCLE;
        }
        var positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, (float) Math.PI, lastLaserReadings.LastPose);
        var negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, (float) -Math.PI, lastLaserReadings.LastPose);
        //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
        var obstacles = new List<Vector2>();
        double positiveUnobstructedRadius = Math.PI,
              negativeUnobstructedRadius  = -Math.PI;
        for (int i = 0; i < lastLaserReadings.Readings.Length; i++) {
            Vector2 rangeBearing = Geometry.ToRangeBearing(lastLaserReadings.Readings[i], lastLaserReadings.LastPose);
            Debug.Log("i " + i + ", r " + rangeBearing.x + ", b " + rangeBearing.y);//TODO: index is bearing in degree!
            if (Math.Abs(rangeBearing.y) > ALPHA) continue;
            if(Geometry.IsWithinFunnel(rangeBearing)) {
                //The obstacle is in front of our possible movements.
                obstacles.Add(rangeBearing);
                if (rangeBearing.x < MIN_OBSTACLE_DISTANCE) {
                    //The obstacle is too close.
                    steering.Halt();
                    //TODO: we will not be able to steer around the obstacle. This means we were unable to steer around the feature or the set position is unreachable from our current position.
                }
                if(rangeBearing.y > 0f) {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], positiveTurningCenter);
                    if (Math.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Math.Atan2(lastLaserReadings.Readings[i].x - positiveTurningCenter.x, lastLaserReadings.Readings[i].z - positiveTurningCenter.y);
                        if (currentAngle < positiveUnobstructedRadius) positiveUnobstructedRadius = currentAngle;
                    }
                } else {
                    var distance = Geometry.EuclideanDistance(lastLaserReadings.Readings[i], negativeTurningCenter);
                    if (Math.Abs(distance - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                        var currentAngle = Math.Atan2(lastLaserReadings.Readings[i].x - negativeTurningCenter.x, lastLaserReadings.Readings[i].z - negativeTurningCenter.y);
                        if (currentAngle > negativeUnobstructedRadius) {
                            negativeUnobstructedRadius = currentAngle;
                        }
                    }
                }
            }
        }
        positiveUnobstructedRadius -= UNOBSTRUCTED_OFFSET;
        negativeUnobstructedRadius += UNOBSTRUCTED_OFFSET;
        //Calculate the segment of circle that the robot has to turn at max steering angle: 
        var bearing = RIGHT_ANGLE - Math.Abs(targetRB.y);
        var c = MainMenu.Physics.turningRadiusSquared + targetRB.x * targetRB.x - 2 * MainMenu.Physics.turningRadius * targetRB.x * Math.Cos(bearing);
        float steeringSegment = (float) (Math.Asin((targetRB.x * Math.Sin(bearing)) / Math.Sqrt(c)) - Math.Asin(Math.Sqrt(c - MainMenu.Physics.turningRadiusSquared) / Math.Sqrt(c)));
        if (bearing < 0) steeringSegment = -steeringSegment;
        //Find the closest steerable way towards the CurrentTargetPosition
        for (float f = 0.0f; f < MAX_OFFSET_ANGLE; f+= OBSTACLE_PLANING_STEP) {
            
            //A steering plan consists of two elements: A line and a turn.
            var plannedTurn = steeringSegment + f;
            if(plannedTurn <= positiveUnobstructedRadius) {
                
            }
            plannedTurn = steeringSegment - f;
            if(plannedTurn >= negativeUnobstructedRadius) {

            }
        }
        if(movingBackwards) {
            lastLaserReadings.LastPose.z -= (float)Math.PI;
            lastLaserReadings.LastPose.z %= FULL_CIRCLE;
        }
    }
}
