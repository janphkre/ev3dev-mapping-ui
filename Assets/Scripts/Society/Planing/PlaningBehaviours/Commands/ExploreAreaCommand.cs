using System.Collections.Generic;
using UnityEngine;

namespace ev3dev.Society {

    class ExploreAreaCommand: AbstractTargetCommand {

        public ExploreAreaCommand(AbstractTargetCommand parentCommand) : base(parentCommand) { }

        public override AbstractTargetCommand PlanMove() {
            Vector2 currentTargetPosition;
            if (!algorithms.globalGraph.GetNewTarget(out currentTargetPosition)) {
                algorithms.SteerStop();
                if (algorithms.globalGraph.HasUnvisitedNodes()) {
                    LinkedList<Vector2> currentPath = algorithms.globalGraph.GetUnexploredNodePath(algorithms.lastLaserReadings.LastPose);
                    return new FollowPathCommand(this, currentPath);
                } else {
                    //The graph does not provide any more unvisited nodes: stop and wait.
                    return parentCommand;
                }
            } else {
                return new ExplorePositionCommand(this, currentTargetPosition);
            }
        }
    }
}
