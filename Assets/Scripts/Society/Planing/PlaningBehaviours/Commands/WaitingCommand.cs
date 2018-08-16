using UnityEngine;

namespace ev3dev.Society {

    class WaitingCommand: AbstractTargetCommand {

        public WaitingCommand(PlaningAlgorithms algorithms) : base(algorithms) { }

		public override AbstractTargetCommand PlanMove() {
            consumeLaserReadings();
            return this;
        }
    }
}
