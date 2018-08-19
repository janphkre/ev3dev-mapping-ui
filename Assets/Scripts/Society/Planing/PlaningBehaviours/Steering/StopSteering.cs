using ev3devMapping;

namespace ev3dev.Society {

    class StopSteering: AbstractSteering {

        protected CarDrive steering;   

        public StopSteering(CarDrive steering) {
            this.steering = steering;
        }

        public override AbstractSteering Execute() {
            steering.Halt();
            return idle();
        }
    }
}
