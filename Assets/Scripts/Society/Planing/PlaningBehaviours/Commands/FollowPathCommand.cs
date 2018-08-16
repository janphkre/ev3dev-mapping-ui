using UnityEngine;
using System.Collections.Generic;

namespace ev3dev.Society {

    class FollowPathCommand: AbstractTargetCommand {

        private LinkedList<Vector2> currentPath;

        public FollowPathCommand(AbstractTargetCommand parentCommand, LinkedList<Vector2> currentPath) : base(parentCommand) {
            this.currentPath = currentPath;
        }

        public override AbstractTargetCommand PlanMove() {
            if (currentPath.Count <= 0) {
                return parentCommand;
            } else {
                Vector2 currentTarget = currentPath.First.Value;
                currentPath.RemoveFirst();
                return new ExplorePositionCommand(this, currentTarget);
            }
        }
    }
}
