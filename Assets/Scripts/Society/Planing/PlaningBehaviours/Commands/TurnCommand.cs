using UnityEngine;
using System.Collections;
using ev3devMapping.Society;
using ev3devMapping;

namespace ev3dev.Society {

    class TurnCommand: AbstractTargetCommand {

        private readonly float targetHeading;
        private readonly bool turnLeft;

        public TurnCommand(AbstractTargetCommand parentCommand, float currentHeading, float turnAngle) : base(parentCommand) {
            this.targetHeading =  Geometry.angleToCircle(currentHeading + turnAngle);
            turnLeft = turnAngle > 0f;
        }

        public override AbstractTargetCommand PlanMove() {
            //TODO!
            PositionData pos = algorithms.positionHistory.GetNewestThreadSafe();
            float currentHeading = pos.heading * Mathf.PI / 180f;
            if (turnLeft) {
                if(currentHeading > targetHeading) {
                    return parentCommand;
                }
            } else {
                if (currentHeading < targetHeading) {
                    return parentCommand;
                }
            }
            consumeLaserReadings();
            return this;
        }
    }
}
