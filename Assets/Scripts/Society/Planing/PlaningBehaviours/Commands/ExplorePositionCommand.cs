using System;
using System.Collections.Generic;
using ev3devMapping;
using ev3devMapping.Society;
using UnityEngine;

namespace ev3dev.Society {

    class ExplorePositionCommand: AbstractTargetCommand {

        private Vector2 currentTargetPosition;
        private Vector3 positiveTurningCenter;//LEFT
        private Vector3 negativeTurningCenter;//RIGHT
        private List<Vector3> obstacles = null;

        public ExplorePositionCommand(AbstractTargetCommand parentCommand, Vector2 currentTargetPosition) : base(parentCommand) {
            this.currentTargetPosition = currentTargetPosition;
        }

		public override AbstractTargetCommand PlanMove() {
            AbstractTargetCommand obstacleCommand = obstaclePlaning();
            algorithms.globalGraph.Feed(algorithms.lastLaserReadings);
            return obstacleCommand;
        }

        private AbstractTargetCommand obstaclePlaning() {
            try {
                var targetRB = Geometry.ToRangeBearing(currentTargetPosition, algorithms.lastLaserReadings.LastPose);
                if (targetRB.x < TARGET_RADIUS) {
                    //Reached the current target.
                    Log.log("Planing - Reached current target.");
                    return parentCommand;
                }
                calculateTurningCenters();
                var unobstructedRadius = findBothUnobstructedRadius();
                if (unobstructedRadius.x <= ZERO_FLOAT || unobstructedRadius.y >= ZERO_FLOAT) {
                    Log.logWarning("Planing - Obstacle hit. (1) " + unobstructedRadius);
                    return backOffObstacle(unobstructedRadius);
                }
                if (algorithms.backwards) {
                    if (Mathf.Abs(targetRB.y) < Geometry.RIGHT_ANGLE) {
                        algorithms.backwards = false;
                        return this;
                    }
                    //Try to turn around in a two/three point turn:
                    AbstractTargetCommand command = hypothesizeTurn(unobstructedRadius);
                    if (command != this) {
                        //backwards = false;
                        return command;
                    }
                    //Continue to go backwards:
                    algorithms.lastLaserReadings.LastPose.z = Geometry.angleToCircle(algorithms.lastLaserReadings.LastPose.z + Geometry.HALF_CIRCLE);
                    targetRB.y = Geometry.angleToCircle(targetRB.y + Geometry.HALF_CIRCLE);
                    unobstructedRadius = findBothUnobstructedRadius();
                    if (unobstructedRadius.x <= ZERO_FLOAT || unobstructedRadius.y >= ZERO_FLOAT) {
                        Log.log("Planing - Obstacle hit. (2) " + unobstructedRadius);
                        return backOffObstacle(unobstructedRadius);
                    }
                }
                Log.log("Target " + currentTargetPosition + ", RB" + targetRB + " Radius" + unobstructedRadius + " Pose" + algorithms.lastLaserReadings.LastPose);
                if (!IsWithinFunnel(targetRB) || Mathf.Abs(targetRB.y) < MIN_CORRECTION_ANGLE) {
                    //The target is not in the current reachable funnel. Move forward.
                    //OR the robot is facing towards the target. No turn is needed.
                    algorithms.steering.Forward(algorithms.backwards);
                } else {
                    float steeringAngle = findSteeringAngle(unobstructedRadius, targetRB);
                    if (float.IsNaN(steeringAngle)) {
                        algorithms.steering.Stop();
                        algorithms.backwards = true;
                        return parentCommand;
                    }
                    algorithms.SteerTurn(steeringAngle);
                }
                return this;
            } finally {
                obstacles = null;
            }
        }

        private void calculateTurningCenters() {
            positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, algorithms.lastLaserReadings.LastPose);
            positiveTurningCenter.z = algorithms.lastLaserReadings.LastPose.z;
            negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, algorithms.lastLaserReadings.LastPose);
            negativeTurningCenter.z = algorithms.lastLaserReadings.LastPose.z;

            algorithms.rendererPositiveTurningCenter.transform.position = (Vector2) positiveTurningCenter;
        }

        //We are in front of an obstacle, so we have to steer around it.
        private AbstractTargetCommand backOffObstacle(Vector2 unobstructedRadius) {
            algorithms.steering.Stop();
            //First we have to find out, if we have to go to the left or right of the obstacle.
            int obstacleIndex = findObstacleIndex(unobstructedRadius);
            int i = obstacleIndex;
            int j = obstacleIndex;
            do {
                i = j;
                j = indexToRange(j + 1, algorithms.lastLaserReadings.ReadingsCount);
                if (Geometry.EuclideanDistance(algorithms.lastLaserReadings.Readings[i], algorithms.lastLaserReadings.Readings[j]) > MIN_OBSTACLE_HOLE_SIZE) {
                    if (checkForHole(i)) {
                        break;
                    }
                }
            } while (j != i);
            float leftDistance = Geometry.EuclideanDistance(algorithms.lastLaserReadings.Readings[i], Vector2.zero);
            int leftIndex = i;
            i = obstacleIndex;
            j = obstacleIndex;
            do {
                i = j;
                j = negativeIndexToRange(j - 1, algorithms.lastLaserReadings.ReadingsCount);
                if (Geometry.EuclideanDistance(algorithms.lastLaserReadings.Readings[i], algorithms.lastLaserReadings.Readings[j]) > MIN_OBSTACLE_HOLE_SIZE) {
                    if (checkForHoleNegative(i)) {
                        break;
                    }
                }
            } while (j != i);
            float rightDistance = Geometry.EuclideanDistance(algorithms.lastLaserReadings.Readings[i], Vector2.zero);
            int rightIndex = i;
            if (leftDistance == ZERO_FLOAT && rightDistance == ZERO_FLOAT) {
                //Reached a dead end.
                Log.log("Planing - Reached dead end. " + unobstructedRadius);
                return parentCommand;
            }

            LinkedList<Vector2> path = new LinkedList<Vector2>();
            float angle;
            if (leftDistance < rightDistance) {
                //Go to the left of the obstacle
                angle = -Geometry.RIGHT_ANGLE;
                path.AddLast(algorithms.lastLaserReadings.Readings[leftIndex]);
            } else {
                //Go to the right of the obstacle
                angle = Geometry.RIGHT_ANGLE;
                path.AddLast(algorithms.lastLaserReadings.Readings[rightIndex]);
            }
            algorithms.SteerTurnBackwards(-angle);
            path.AddLast(currentTargetPosition);
            return new TurnCommand(new FollowPathCommand(this, path));
        }   

        private int findObstacleIndex(Vector2 unobstructedRadius) {
            unobstructedRadius.x += UNOBSTRUCTED_OFFSET;
            unobstructedRadius.y = UNOBSTRUCTED_OFFSET - unobstructedRadius.y;
            for (int i = 0; i < algorithms.lastLaserReadings.ReadingsCount; i++) {
                if (Mathf.Abs(algorithms.lastLaserReadings.ReadingsRB[i].y) > BETA) continue;
                if (IsWithinFunnel(algorithms.lastLaserReadings.ReadingsRB[i])) {
                    if (algorithms.lastLaserReadings.ReadingsRB[i].x < MIN_OBSTACLE_DISTANCE) {
                        return i;
                    }
                    //The obstacle is in front of our possible movements.
                    obstacles.Add(algorithms.lastLaserReadings.Readings[i]);
                    if (algorithms.lastLaserReadings.ReadingsRB[i].y >= ZERO_FLOAT) {
                        var turningRB = Geometry.ToRangeBearing2(algorithms.lastLaserReadings.Readings[i], positiveTurningCenter);
                        if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                            turningRB.y = Geometry.RIGHT_ANGLE + turningRB.y;
                            if (Math.Abs(turningRB.y - unobstructedRadius.x) <= EPSILON) return i;
                        }
                    } else {
                        var turningRB = Geometry.ToRangeBearing2(algorithms.lastLaserReadings.Readings[i], negativeTurningCenter);
                        if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                            turningRB.y = Geometry.RIGHT_ANGLE - turningRB.y;
                            if (Math.Abs(turningRB.y - unobstructedRadius.y) <= EPSILON) return i;
                        }
                    }
                }
            }
            throw new ArgumentOutOfRangeException("No index found for " + unobstructedRadius);
        }

        private bool checkForHole(int index) {
            for (int k = 2; k < algorithms.lastLaserReadings.ReadingsCount / 4; k++) {
                int l = indexToRange(index + k, algorithms.lastLaserReadings.ReadingsCount);
                if (Geometry.EuclideanDistance(algorithms.lastLaserReadings.Readings[index], algorithms.lastLaserReadings.Readings[l]) < MIN_OBSTACLE_HOLE_SIZE) {
                    return false;
                }
            }
            return true;
        }

        private bool checkForHoleNegative(int index) {
            for (int k = -2; k > -algorithms.lastLaserReadings.ReadingsCount / 4; k--) {
                int l = negativeIndexToRange(index + k, algorithms.lastLaserReadings.ReadingsCount);
                if (Geometry.EuclideanDistance(algorithms.lastLaserReadings.Readings[index],
                                               algorithms.lastLaserReadings.Readings[l]) < MIN_OBSTACLE_HOLE_SIZE) {
                    return false;
                }
            }
            return true;
        }

        /*
         * Tries to turn the vehicle around. Input is an unobstructedRadius for left and right hand turns.
         * The lastLaserReadings are used as an input as well.
         * Returns true if a turn is possible and will be executed,
         *         false otherwise.
         */
        private AbstractTargetCommand hypothesizeTurn(Vector2 unobstructedRadius) {
            if (unobstructedRadius.x >= Geometry.RIGHT_ANGLE) {
                if (unobstructedRadius.x >= Geometry.HALF_CIRCLE) {
                    return turn(Geometry.HALF_CIRCLE);
                }
                Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, Geometry.EIGHTH_CIRCLE, algorithms.lastLaserReadings.LastPose);
                movedPose.z = Geometry.angleToCircle(algorithms.lastLaserReadings.LastPose.z - Geometry.RIGHT_ANGLE);
                if (findSingleUnobstructedRadius(movedPose, Geometry.RIGHT_ANGLE) >= Geometry.RIGHT_ANGLE) {
                    return turn(Geometry.RIGHT_ANGLE);
                }
            }
            if (unobstructedRadius.y <= -Geometry.RIGHT_ANGLE) {
                if (unobstructedRadius.y <= -Geometry.HALF_CIRCLE) {
                    return turn(-Geometry.HALF_CIRCLE);
                }
                Vector3 movedPose = Geometry.FromRangeBearing(MainMenu.Physics.turningRadiusAngledSquared, -Geometry.EIGHTH_CIRCLE, algorithms.lastLaserReadings.LastPose);
                if (findSingleUnobstructedRadius(movedPose, -Geometry.RIGHT_ANGLE) >= Geometry.RIGHT_ANGLE) {
                    return turn(-Geometry.RIGHT_ANGLE);
                }
            }
            //TODO: If the above failed, we may be able to turn around by reversing first. This would mean code changes to the TargetCommand.Turn as well.
            // Skip that for now for better performance in narrow situations.
            //TODO: If the above succeds we might get stuck because the turn does not check what happens after the turn.
            return this;
        }

        private AbstractTargetCommand turn(float angle) {
            Log.log("Planing - turning with angle "+angle);
            algorithms.SteerTurnForward(angle);
            return new TurnCommand(this);
        }

        private Vector2 findBothUnobstructedRadius() {
            obstacles = new List<Vector3>();
            //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
            Vector2 unobstructedRadius = new Vector2(ALPHA, ALPHA);
            for (int i = 0; i < algorithms.lastLaserReadings.ReadingsCount; i++) {
                if (Mathf.Abs(algorithms.lastLaserReadings.ReadingsRB[i].y) > BETA) continue;
                if (IsWithinFunnel(algorithms.lastLaserReadings.ReadingsRB[i])) {
                    if (algorithms.lastLaserReadings.ReadingsRB[i].x < MIN_OBSTACLE_DISTANCE) {
                        //The obstacle is too close.
                        //We will not be able to steer around the obstacle. This means we were unable to steer around the feature or the set position is unreachable from our current position.
                        return Vector2.zero;
                    }
                    //The obstacle is in front of our possible movements.
                    obstacles.Add(algorithms.lastLaserReadings.Readings[i]);
                    if (algorithms.lastLaserReadings.ReadingsRB[i].y >= ZERO_FLOAT) {
                        var turningRB = Geometry.ToRangeBearing2(algorithms.lastLaserReadings.Readings[i], positiveTurningCenter);
                        if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                            turningRB.y = Geometry.RIGHT_ANGLE + turningRB.y;
                            if (turningRB.y < unobstructedRadius.x) unobstructedRadius.x = turningRB.y;
                        }
                    } else {
                        var turningRB = Geometry.ToRangeBearing2(algorithms.lastLaserReadings.Readings[i], negativeTurningCenter);
                        if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                            turningRB.y = Geometry.RIGHT_ANGLE - turningRB.y;
                            if (turningRB.y < unobstructedRadius.y) unobstructedRadius.y = turningRB.y;
                        }
                    }
                }
            }
            unobstructedRadius.x -= UNOBSTRUCTED_OFFSET;
            unobstructedRadius.y = UNOBSTRUCTED_OFFSET - unobstructedRadius.y;
            Log.log("Unobstructed Radius: " + unobstructedRadius);
            DrawArc(algorithms.rendererX, UNOBSTRUCTED_HEIGHT, positiveTurningCenter, -Geometry.RIGHT_ANGLE + algorithms.lastLaserReadings.LastPose.z, unobstructedRadius.x);
            DrawArc(algorithms.rendererY, UNOBSTRUCTED_HEIGHT, negativeTurningCenter, Geometry.RIGHT_ANGLE + algorithms.lastLaserReadings.LastPose.z, unobstructedRadius.y);
            return unobstructedRadius;
        }

        private float findSingleUnobstructedRadius(Vector3 origin, float angle) {
            Vector2 turningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, angle, origin);
            //The (potential) obstacles are within the funnel that can be reached by the vehicle excluding everything closer than the turn radius and everything behind the vehicle.
            float unobstructedRadius = Geometry.HALF_CIRCLE;
            float isNegative = angle < 0 ? -1f : 1f;
            for (int i = 0; i < algorithms.lastLaserReadings.ReadingsCount; i++) {
                Vector2 rangeBearing = Geometry.ToRangeBearing(algorithms.lastLaserReadings.Readings[i], origin);
                if (Mathf.Abs(rangeBearing.y) > Geometry.HALF_CIRCLE) continue;
                if (IsWithinFunnel(rangeBearing)) {
                    //The obstacle is in front of our possible movements.
                    rangeBearing.y *= isNegative;
                    if (rangeBearing.y >= ZERO_FLOAT) {
                        var turningRB = Geometry.ToRangeBearing2(algorithms.lastLaserReadings.Readings[i], turningCenter);
                        if (Mathf.Abs(turningRB.x - MainMenu.Physics.turningRadius) < MIN_OBSTACLE_DISTANCE) {
                            var currentAngle = Mathf.Asin(rangeBearing.x * Mathf.Cos(rangeBearing.y) / turningRB.x);
                            currentAngle = Geometry.angleToPositiveCircle(currentAngle);
                            if (currentAngle < unobstructedRadius) unobstructedRadius = currentAngle;
                        }
                    }
                }
            }
            return unobstructedRadius - UNOBSTRUCTED_OFFSET * isNegative;
        }

        private float findSteeringAngle(Vector2 unobstructedRadius, Vector2 targetRB) {
            //Calculate the segment of circle that the robot has to turn at max steering angle: 
            var bearing = Geometry.RIGHT_ANGLE - Mathf.Abs(targetRB.y);
            var c = MainMenu.Physics.turningRadiusSquared + targetRB.x * targetRB.x - 2f * MainMenu.Physics.turningRadius * targetRB.x * Mathf.Cos(bearing);
            float steeringSegment = (Mathf.Asin((targetRB.x * Mathf.Sin(bearing)) / Mathf.Sqrt(c)) - Mathf.Asin(Mathf.Sqrt(c - MainMenu.Physics.turningRadiusSquared) / Mathf.Sqrt(c)));
            if (bearing < 0) steeringSegment = -steeringSegment;
            //Find the closest steerable way towards the currentTargetPosition
            //A steering plan consists of two elements: A line and a turn.
            if (steeringSegment > unobstructedRadius.x) steeringSegment = unobstructedRadius.x;
            else if (steeringSegment < unobstructedRadius.y) steeringSegment = unobstructedRadius.y;
            else {
                float f = ZERO_FLOAT;
                for (; f < MAX_OFFSET_ANGLE; f += OBSTACLE_PLANING_STEP) {
                    var currentAngle = steeringSegment + f;
                    if (currentAngle <= unobstructedRadius.x) {
                        if (currentAngle >= unobstructedRadius.y) {
                            Vector3 line = Geometry.Rotate(algorithms.lastLaserReadings.LastPose, currentAngle >= ZERO_FLOAT ? positiveTurningCenter : negativeTurningCenter, currentAngle);
                            line.z = algorithms.lastLaserReadings.LastPose.z + currentAngle;
                            //targetRB.x is roughly the real target distance from line plus not more than MainMenu.Physics.turningRadius.
                            if (checkLine(line, 0.1f)) break;
                        } else {
                            f = unobstructedRadius.y - steeringSegment - OBSTACLE_PLANING_STEP;
                        }
                    } else {
                        f = 2f * MAX_OFFSET_ANGLE;
                        break;
                    }
                }
                float g = OBSTACLE_PLANING_STEP;
                for (; g < f; g += OBSTACLE_PLANING_STEP) {
                    var currentAngle = steeringSegment - g;
                    if (currentAngle >= unobstructedRadius.y) {
                        if (currentAngle <= unobstructedRadius.x) {
                            Vector3 line = Geometry.Rotate(algorithms.lastLaserReadings.LastPose, currentAngle >= ZERO_FLOAT ? positiveTurningCenter : negativeTurningCenter, currentAngle);
                            line.z = algorithms.lastLaserReadings.LastPose.z + currentAngle;
                            if (checkLine(line, 0.1f)) break;
                        } else {
                            g = steeringSegment - unobstructedRadius.x - OBSTACLE_PLANING_STEP;
                        }
                    } else {
                        g = 2f * MAX_OFFSET_ANGLE;
                        break;
                    }
                }
                if (f >= MAX_OFFSET_ANGLE && g >= MAX_OFFSET_ANGLE) {
                    //This means that either the steerable range was smaller than OBSTACLE_PLANING_STEP
                    //or the target is blocked by an obstacle -> remove edge from graph
                    algorithms.globalGraph.DisconnectNode(currentTargetPosition);
                    Log.log("Disconnecting " + currentTargetPosition);
                    return float.NaN;
                }
                steeringSegment += (f < g ? f : g);
            }
            if (steeringSegment > ZERO_FLOAT) DrawArc(algorithms.rendererSteering, STEERING_HEIGHT, positiveTurningCenter, -Geometry.RIGHT_ANGLE + algorithms.lastLaserReadings.LastPose.z, steeringSegment);
            else DrawArc(algorithms.rendererSteering, STEERING_HEIGHT, negativeTurningCenter, Geometry.RIGHT_ANGLE + algorithms.lastLaserReadings.LastPose.z, steeringSegment);
            return steeringSegment / MainMenu.Physics.turningRadius;
        }

        private bool checkLine(Vector3 line, float length) {
            foreach (Vector3 obstacle in obstacles) {
                Vector2 obstacleRB = Geometry.ToRangeBearing(obstacle, line);
                if (Mathf.Abs(Mathf.Sin(obstacleRB.y) * obstacleRB.x) < MIN_OBSTACLE_DISTANCE && Mathf.Abs(Mathf.Cos(obstacleRB.y) * obstacleRB.x) < length) return false;
            }
            return true;
        }

        //Checks wether the feature, provided as range and bearing, is within the currently drivable funnel of the robot
        public static bool IsWithinFunnel(Vector2 featureRB) {
            if (featureRB.y == ZERO_FLOAT) return true;
            return Mathf.Sin(Mathf.Abs(featureRB.y)) * MainMenu.Physics.turningDiameter - MainMenu.Physics.halfWheelbase < featureRB.x;
            //This is not completly correct. The turning circle should only be moved half wheelbase to the side!
        }

        public static void DrawArc(LineRenderer renderer, float height, Vector2 center, float offset, float angle) {
            var arcPoints = new List<Vector3>();
            float sign = Mathf.Sign(angle);
            angle *= sign;
            for (float f = ZERO_FLOAT; f < angle; f += ARC_STEP) {
                float x = Mathf.Cos(sign * f + offset) * MainMenu.Physics.turningRadius;
                float y = Mathf.Sin(sign * f + offset) * MainMenu.Physics.turningRadius;
                arcPoints.Add(new Vector3(x + center.x, height, y + center.y));
            }
            renderer.positionCount = arcPoints.Count;
            renderer.SetPositions(arcPoints.ToArray());
        }

        public static int indexToRange(int index, int range) {
            return index % range;
        }

        public static int negativeIndexToRange(int index, int range) {
            return ((index % range) + range) % range;
        }
    }
}
