using ev3devMapping;

namespace ev3dev.Society {

    class ForwardSteering: AbstractSteering {

        public ForwardSteering(CarDrive steering, bool backwards) {
            steering.DriveAhead(backwards);
        }

        public override AbstractSteering Execute() {
            return idle();
        }
    }
}
