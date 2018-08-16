namespace ev3dev.Society {
    class TargetCommandWrapper {

        private PlaningAlgorithms algorithms;
        private object currentTargetCommandLock = new object();
        private AbstractTargetCommand currentTargetCommand;

        public TargetCommandWrapper(PlaningAlgorithms algorithms) {
            this.algorithms = algorithms;
            currentTargetCommand = new WaitingCommand(algorithms);
        }

        public void ExecuteStep() {
            lock (currentTargetCommandLock) {
                AbstractTargetCommand command = currentTargetCommand.PlanMove();
                if (command == currentTargetCommand) {
                    return;
                }

                currentTargetCommand = command;
            }
            ExecuteStep();
        }

        public void SetStartCommand() {
            lock (currentTargetCommandLock) {
                algorithms.steering.Halt();
                algorithms.backwards = false;
                currentTargetCommand = new ExploreAreaCommand(new WaitingCommand(algorithms));
            }
        }

        public void SetReturnToStartCommand() {
            lock (currentTargetCommandLock) {
                algorithms.steering.Halt();
                algorithms.backwards = !algorithms.backwards;
                currentTargetCommand = new FollowPathCommand(new WaitingCommand(algorithms), algorithms.globalGraph.GetStartPath(algorithms.positionHistory.GetNewestThreadSafe().position));
            }
        }

        public AbstractTargetCommand GetCurrentTargetCommand() {
            lock (currentTargetCommandLock) {
                return currentTargetCommand;
            }
        }


    }
}
