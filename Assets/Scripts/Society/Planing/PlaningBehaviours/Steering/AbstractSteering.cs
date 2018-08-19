namespace ev3dev.Society {

    abstract class AbstractSteering {
        public abstract AbstractSteering Execute();

        protected virtual AbstractSteering idle() {
            return new IdleSteering();
        }
    }
}
