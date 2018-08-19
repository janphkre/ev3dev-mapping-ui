using ev3devMapping;
using System;

namespace ev3dev.Society {

    class TurnStopSteering: TurnSteering { 

        public TurnStopSteering(CarDrive steering, PositionHistory positionHistory, float turnAngle, bool backwards, Func<float, bool> isHeadingReached) : base(steering, positionHistory, turnAngle, backwards, isHeadingReached) { }

        protected override AbstractSteering idle() {
            return new StopSteering(steering);
        }
    }
}