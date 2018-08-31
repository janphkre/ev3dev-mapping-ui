using ev3dev.Society;
using ev3devMapping.Society;
using Superbest_random;
using UnityEngine;
using UnityEngine.Assertions;

namespace ev3devMapping.Testing {

    //This script is attached to the "Settings" GameObject.
    public class TestingReconning: ITesting {

        private const float STEP_SIZE = 0.05f;
        private const int MAX_HEADING = 18000;
        private CarReconning reconning = null;
        private CarReconningPacket testPacket = new CarReconningPacket();

        private System.Random random = new System.Random();
        private AbstractSteering turnSteering = null;
        private PositionHistory positionHistory = null;
        private float currentHeading = 0f;
        private float targetHeading = 0f;
        private bool turningBackwards = false;

        private MockCarDrive.State expectedSteeringState = MockCarDrive.State.HALTED;
        private MockCarDrive mockCarDrive = null;

        private LineRenderer rendererX;
        private LineRenderer rendererY;

        public void Test(GameObject robot) {
            MainMenu.Physics = new Physics();
            MainMenu.Physics.turningRadius = 0.595f;
            MainMenu.Physics.wheelbaseMm = 185.0f;
            MainMenu.Physics.Calculate();

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

            reconning = robot.transform.Find("CarReconning").GetComponent<CarReconning>();
            positionHistory = new PositionHistory(10);
        }

        void Update() {
            if (reconning != null) {
                StepReconning();
            }
            StepTurnSteering();
        }

        private void StepReconning() {
            testPacket.timestamp_us = Timestamp.TimestampUs();
            var position = reconning.TestProcessPacket(testPacket);
            testPacket.position_drive += (int)(2f * STEP_SIZE * Constants.MM_IN_M / MainMenu.Physics.distancePerEncoderCountMm);
            var headingDelta = STEP_SIZE / MainMenu.Physics.turningRadius;
            testPacket.heading += (short)(headingDelta * 180f * 100f / Mathf.PI);
            if (testPacket.heading > MAX_HEADING) {
                testPacket.heading = (short)((int)testPacket.heading - 2 * MAX_HEADING);
            } else if (testPacket.heading < -MAX_HEADING) {
                testPacket.heading = (short)((int)testPacket.heading + 2 * MAX_HEADING);
            }
            var lastPose = new Vector3(position.position.x, position.position.z, position.heading);
            var positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, lastPose);
            var negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, lastPose);
            ExplorePositionCommand.DrawArc(rendererX, 0.2f, positiveTurningCenter, -Geometry.RIGHT_ANGLE + lastPose.z, Geometry.HALF_CIRCLE);
            ExplorePositionCommand.DrawArc(rendererX, 0.2f, negativeTurningCenter, Geometry.RIGHT_ANGLE + lastPose.z, -Geometry.HALF_CIRCLE);
        }

        private void StepTurnSteering() {
            if (turnSteering == null) {
                SetupTurnIteration();
            }
            bool hasMoreHeadings = GetNextCurrentHeading();
            PutToHistory(currentHeading);
            var result = turnSteering.Execute();

            if (hasMoreHeadings) {
                var currentState = mockCarDrive.GetState();
                if (expectedSteeringState != currentState) {
                    Debug.LogError("Expected " + expectedSteeringState.ToString() + "Got " + currentState.ToString());
                }
            } else {
                if (!(result is StopSteering)) {
                    Debug.LogError("Expected StopSteering, Got " + result.ToString());
                }
                var currentState = mockCarDrive.GetState();
                if (MockCarDrive.State.FORWARD != currentState) {
                    Debug.LogError("Expected " + MockCarDrive.State.FORWARD + "Got " + currentState.ToString());
                }
                Debug.Log("Turn succeeded");
                turnSteering = null;
            }

        }

        private void SetupTurnIteration() {
            currentHeading = Random.Range(0f, Geometry.FULL_CIRCLE);
            targetHeading = Geometry.angleToCircle(currentHeading + Random.Range(1f, Geometry.FULL_CIRCLE));
            if (targetHeading > Geometry.HALF_CIRCLE) {
                targetHeading = Geometry.FULL_CIRCLE - targetHeading;
            }
            PutToHistory(currentHeading);
            float turnAngle = targetHeading - currentHeading;
            mockCarDrive = new MockCarDrive();
            turningBackwards = random.NextBoolean();
            turnSteering = new TurnStopSteering(mockCarDrive, positionHistory, turnAngle, turningBackwards, TurnObserver.getHeadingComparison(currentHeading, turnAngle, turningBackwards));
            expectedSteeringState = turnAngle > 0 ? MockCarDrive.State.STEERING_LEFT : MockCarDrive.State.STEERING_RIGHT;
        }

        private bool GetNextCurrentHeading() {
            if (targetHeading < currentHeading) {
                currentHeading -= STEP_SIZE;
                if (targetHeading >= currentHeading) {
                    return false;
                }
            } else {
                currentHeading += STEP_SIZE;
                if (targetHeading <= currentHeading) {
                    return false;
                }
            }
            return true;
        }

        private void PutToHistory(float heading) {
            positionHistory.PutThreadSafe(new PositionData() {
                timestamp = Timestamp.TimestampUs(),
                position = Vector3.zero,
                heading = heading * 180f / Geometry.HALF_CIRCLE
            });
        }

        internal class MockCarDrive: ICarSteering {

            public enum State {
                HALTED, STEERING_LEFT, STEERING_RIGHT, FORWARD
            }

            public State currentState = State.HALTED;

            public State GetState() {
                return currentState;
            }

            public void Halt() {
                currentState = State.HALTED;
            }

            public void Steer(float segment, bool backwards) {
                currentState = segment > 0 ? State.STEERING_LEFT : State.STEERING_RIGHT;
            }

            //Moves forward for the segment of the max steering angle circle and stops afterwards.
            public void SteerForward(float segment) {
                currentState = segment > 0 ? State.STEERING_LEFT : State.STEERING_RIGHT;

            }

            //Same as SteerForward but backwards
            public void SteerBackward(float segment) {
                currentState = segment > 0 ? State.STEERING_LEFT : State.STEERING_RIGHT;

            }

            public void DriveAhead(bool backwards) {
                currentState = State.FORWARD;
            }
        }
    }

}
