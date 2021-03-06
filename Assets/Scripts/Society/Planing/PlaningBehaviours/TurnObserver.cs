using UnityEngine;
using System.Collections;
using ev3devMapping;
using ev3devMapping.Society;
using System;

namespace ev3dev.Society {

    class TurnObserver {

        private CarDrive steering;
        private PositionHistory positionHistory;

        private object requestedCommandLock = new object();
        private volatile AbstractSteering requestedCommand = new IdleSteering();

        public TurnObserver(CarDrive steering, PositionHistory positionHistory) {
            this.steering = steering;
            this.positionHistory = positionHistory;
        }

        public IEnumerator turnRoutine() {
            while (true) {
                yield return new WaitWhile(() => requestedCommand is IdleSteering);
                AbstractSteering currentCommand = requestedCommand;
                AbstractSteering resultCommand = currentCommand.Execute();
                if(resultCommand != currentCommand) {
                    lock(requestedCommandLock) {
                        if(requestedCommand == currentCommand) {
                            requestedCommand = resultCommand;
                        }
                    }
                }
            }
        }

        public void Forward(bool backwards) {
            AbstractSteering forwardSteering = new ForwardSteering(steering, backwards);
            Log.log("ForwardSteering(b "+ backwards +")");
            lock(requestedCommandLock) {
                requestedCommand = forwardSteering;
            }
        }

        public void Turn(float currentHeading, float turnAngle, bool backwards) {
            AbstractSteering turnSteering = new TurnSteering(steering, positionHistory, turnAngle, backwards, getHeadingComparison(currentHeading, turnAngle, backwards));
            Log.log("TurnSteering(cH "+ currentHeading + ", tA " + turnAngle + ", b " + backwards +")");
            lock(requestedCommandLock) {
                requestedCommand = turnSteering;
            }
        }

        public void TurnStop(float currentHeading, float turnAngle, bool backwards) {
            AbstractSteering turnStopSteering = new TurnStopSteering(steering, positionHistory, turnAngle, backwards, getHeadingComparison(currentHeading, turnAngle, backwards));
            Log.log("TurnStopSteering(cH "+ currentHeading + ", tA " + turnAngle + ", b " + backwards +")");
            lock(requestedCommandLock) {
                requestedCommand = turnStopSteering;
            }
        }

        public void Stop() {
            AbstractSteering stopSteering = new StopSteering(steering);
            Log.log("StopSteering");
            lock(requestedCommandLock) {
                requestedCommand = stopSteering;
            }
        }

        public bool isIdle() {
            return requestedCommand is IdleSteering;
        }

        public static Func<float, bool> getHeadingComparison(float currentHeading, float turnAngle, bool backwards) {
            float targetHeadingDegrees = Geometry.angleToCircle(currentHeading + turnAngle) * 180f / Geometry.HALF_CIRCLE;
            float currentHeadingDegrees = currentHeading * 180f / Geometry.HALF_CIRCLE;
            if(turnAngle > 0) {
                return turnAngleDegrees => turnAngleDegrees >= targetHeadingDegrees || turnAngleDegrees < currentHeadingDegrees;
            } else {
                return turnAngleDegrees => turnAngleDegrees <= targetHeadingDegrees || turnAngleDegrees > currentHeadingDegrees;
            }
        }
    }
}
