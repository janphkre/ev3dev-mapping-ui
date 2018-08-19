namespace ev3dev.Society {
    class TargetCommandWrapper {

        private PlaningAlgorithms algorithms;
        private object currentTargetCommandLock = new object();
        private volatile AbstractTargetCommand currentTargetCommand;

        public TargetCommandWrapper(PlaningAlgorithms algorithms) {
            this.algorithms = algorithms;
            currentTargetCommand = new WaitingCommand(algorithms);
        }

        public void ExecuteStep() {
            lock (currentTargetCommandLock) {
                if(!(currentTargetCommand is WaitingCommand)) {
                    emptyMethod();
                }
                AbstractTargetCommand command = currentTargetCommand.PlanMove();
                if (command == currentTargetCommand) {
                    return;
                }
                Log.log("Command: " + currentTargetCommand.ToString());
                currentTargetCommand = command;
            }
            ExecuteStep();
        }

        private void emptyMethod() { }

        public void SetStartCommand() {
            lock (currentTargetCommandLock) {
                algorithms.backwards = false;
                Log.log("Command: ExploreAreaCommand");
                currentTargetCommand = new ExploreAreaCommand(new WaitingCommand(algorithms));
            }
        }

        public void SetReturnToStartCommand() {
            lock (currentTargetCommandLock) {
                algorithms.SteerStop();
                algorithms.backwards = !algorithms.backwards;
                Log.log("Command: FollowPathCommand(pathToStart)");
                currentTargetCommand = new FollowPathCommand(new WaitingCommand(algorithms), algorithms.globalGraph.GetStartPath(algorithms.lastLaserReadings.LastPose));
            }
        }

        public AbstractTargetCommand GetCurrentTargetCommand() {
            lock (currentTargetCommandLock) {
                return currentTargetCommand;
            }
        }
    }
}
