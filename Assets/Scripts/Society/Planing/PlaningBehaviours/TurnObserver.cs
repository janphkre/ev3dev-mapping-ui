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

        public void turn(float currentHeading, float turnAngle, bool backwards) {
            lock(requestedCommandLock) {
                requestedCommand = new TurnSteering(steering, positionHistory, turnAngle, backwards, getHeadingComparison(currentHeading, turnAngle, backwards));
            }
        }

        public void turnStop(float currentHeading, float turnAngle, bool backwards) {
            lock(requestedCommandLock) {
                requestedCommand = new TurnStopSteering(steering, positionHistory, turnAngle, backwards, getHeadingComparison(currentHeading, turnAngle, backwards));
            }
        }

        public void stop() {
            lock(requestedCommandLock) {
                requestedCommand = new StopSteering(steering);
            }
        }

        private Func<float, bool> getHeadingComparison(float currentHeading, float turnAngle, bool backwards) {
            if(backwards) {
                turnAngle *= -1;
            }
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
