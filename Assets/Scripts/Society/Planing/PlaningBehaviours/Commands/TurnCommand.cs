
namespace ev3dev.Society {

    class TurnCommand: AbstractTargetCommand {

        public TurnCommand(AbstractTargetCommand parentCommand) : base(parentCommand) { }

        public override AbstractTargetCommand PlanMove() {
            if(algorithms.isSteeringIdle()) {
                return parentCommand;
            }
            consumeLaserReadings();
            return this;
        }
    }
}
