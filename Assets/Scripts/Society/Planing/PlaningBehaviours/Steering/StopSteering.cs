using ev3devMapping;

namespace ev3dev.Society {

    class StopSteering: AbstractSteering {

        protected ICarSteering steering;   

        public StopSteering(ICarSteering steering) {
            this.steering = steering;
        }

        public override AbstractSteering Execute() {
            steering.Halt();
            return idle();
        }
    }
}
