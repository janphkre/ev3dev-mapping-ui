using System;
using ev3devMapping;

namespace ev3dev.Society {

    class TurnSteering: AbstractSteering {

        protected CarDrive steering;   
        protected PositionHistory positionHistory;
        private Func<float, bool> isHeadingReached;
        private bool backwards;

        public TurnSteering(CarDrive steering, PositionHistory positionHistory, float turnAngle, bool backwards, Func<float, bool> isHeadingReached) {
            this.steering = steering;
            this.positionHistory = positionHistory;
            this.backwards = backwards;
            this.isHeadingReached = isHeadingReached;

            steering.Steer(turnAngle, backwards);
        }

        public override AbstractSteering Execute() {
            PositionData pos = positionHistory.GetNewestThreadSafe();
            if(isHeadingReached.Invoke(pos.heading)) {
                steering.DriveAhead(backwards);
                return idle();
            }
            return this;
        }
    }
}
