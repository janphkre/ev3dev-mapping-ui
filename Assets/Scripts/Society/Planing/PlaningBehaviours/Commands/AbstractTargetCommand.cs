using ev3devMapping.Society;
using UnityEngine;

namespace ev3dev.Society {

    class PlaningAlgorithms {
        public TurnObserver steering;
        public Graph globalGraph;

        public PlaningInputData lastLaserReadings = null;

        public bool backwards = false;

        public LineRenderer rendererX;
        public LineRenderer rendererY;
        public LineRenderer rendererSteering;
        public GameObject rendererPositiveTurningCenter;

        public void SteerForward() {
            steering.Forward(backwards);
        }

        public void SteerTurn(float angle) {
            steering.Turn(lastLaserReadings.LastPose.z, angle, backwards);
        }

        public void SteerTurnStop(float angle) {
            steering.TurnStop(lastLaserReadings.LastPose.z, angle, backwards);
        }

        public void SteerTurnForward(float angle) {
            steering.TurnStop(lastLaserReadings.LastPose.z, angle, false);
        }

        public void SteerTurnBackwards(float angle) {
            steering.TurnStop(lastLaserReadings.LastPose.z, angle, true);
        }

        public void SteerStop() {
            steering.Stop();
        }

        public bool isSteeringIdle() {
            return steering.isIdle();
        }
    };

    abstract class AbstractTargetCommand {
        //Parameters:
        public const float ALPHA = Geometry.HALF_CIRCLE + Geometry.HALF_CIRCLE / 4f;
        public const float BETA = Geometry.RIGHT_ANGLE + Geometry.RIGHT_ANGLE / 3f;
        public const float GAMMA = Geometry.RIGHT_ANGLE / 8f;
        public const float OBSTACLE_PLANING_STEP = Geometry.HALF_CIRCLE / 36f;
        public const float MIN_OBSTACLE_DISTANCE = 0.11f;
        public const float UNOBSTRUCTED_OBSTACLE_MULTIPLIER = 1f;
        public const float TARGET_RADIUS = 0.1f;
        public const float MAX_OFFSET_ANGLE = Geometry.HALF_CIRCLE;
        public const float MIN_CORRECTION_ANGLE = 0.04f;
        public const float ARC_STEP = 1f / 4f;
        public const float UNOBSTRUCTED_HEIGHT = 0.02f;
        public const float STEERING_HEIGHT = 0.04f;
        public const float MIN_OBSTACLE_HOLE_SIZE = MIN_OBSTACLE_DISTANCE * 2.2f;
        public const float EPSILON = 0.0001f;
        protected const float ZERO_FLOAT = 0.0f;

        //Calculated once at Startup:
        public static float UNOBSTRUCTED_OFFSET;

        protected AbstractTargetCommand parentCommand;
        protected PlaningAlgorithms algorithms;

        public AbstractTargetCommand(AbstractTargetCommand parentCommand) {
            this.parentCommand = parentCommand;
            algorithms = parentCommand.algorithms;
        }

        public AbstractTargetCommand(PlaningAlgorithms algorithms) {
            parentCommand = null;
            this.algorithms = algorithms;
        }

        public abstract AbstractTargetCommand PlanMove();

        protected void consumeLaserReadings() {
            algorithms.globalGraph.Feed(algorithms.lastLaserReadings);
        }
    }
}