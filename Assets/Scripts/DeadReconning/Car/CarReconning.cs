using ev3devMapping.Society;
using System;
using UnityEngine;

namespace ev3devMapping {

[Serializable]
public class CarReconningModuleProperties: ModuleProperties {
    public int pollMs = 10;
}

class CarReconning: ReplayableUDPServer<CarReconningPacket> {

        private const float MS_IN_US = 0.001f;
        private const float DRIVE_STOP_CUTOFF = 0.01f;
        private const float HEADING_FORWARD_CUTOFF = 1.0f;
        private const int STEER_CIRCLE_CUTOFF = 2;

    public CarReconningModuleProperties module = null;
    
    private CarReconningPacket lastPacket = new CarReconningPacket();
    private PositionData lastPosition = new PositionData();
    private Vector3 previousPosition = Vector3.zero;
    private float initialHeading;
    private int headingDrift = 0;
    
    protected override void Awake() {
        base.Awake();
        lastPosition = new PositionData { position = transform.parent.position, heading = transform.parent.eulerAngles.y };
        initialHeading = lastPosition.heading;
    }

    protected override void Start() {
        base.Start();
    }
    
    protected void Update() {
        PositionData currentPosition;
        try {
            currentPosition = positionHistory.GetNewestThreadSafe();
        } catch(InvalidOperationException) {
            return;
        }
        transform.parent.transform.position = currentPosition.position;
		transform.parent.transform.rotation = Quaternion.AngleAxis(-currentPosition.heading, Vector3.up);
            if(!currentPosition.position.Equals(previousPosition)) {
                GameObject o = GameObject.CreatePrimitive(PrimitiveType.Cube);
                o.transform.position = currentPosition.position;
                o.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                o.GetComponent<MeshRenderer>().material.color = Color.red;
                previousPosition = currentPosition.position;
            }
        
    }

        protected override void ProcessPacket(CarReconningPacket packet) {
            //First call - set first udp packet with reference encoder positions
            if (lastPacket.timestamp_us == 0) {
                lastPacket.CloneFrom(packet);
                return;
            }
            ulong timeDiffUs = packet.timestamp_us - lastPacket.timestamp_us;
            // UDP doesn't guarantee ordering of packets, if previous odometry is newer ignore the received
            if (timeDiffUs <= 0) {
                print("car-reconning - ignoring out of time packet (previous, now):" + Environment.NewLine + lastPacket.ToString() + Environment.NewLine + packet.ToString());
                return;
            }

            lastPosition.timestamp = packet.timestamp_us;
            EstimateNewPosition(packet, timeDiffUs);
            lastPacket.CloneFrom(packet);
        }

        private void EstimateNewPosition(CarReconningPacket packet, ulong timeDiffUs) {
            // Calculate the motor displacement since last packet:
            float driveDiff = packet.position_drive - lastPacket.position_drive;

            if (Mathf.Abs(driveDiff) < DRIVE_STOP_CUTOFF) {
                //remove drift from heading:
                headingDrift += packet.heading - lastPacket.heading;
            }
            float headingInDegrees = ((headingDrift - packet.heading) / 100.0f) + initialHeading;

            if (Mathf.Abs(driveDiff) < DRIVE_STOP_CUTOFF) {
                EstimatePositionWithStop(headingInDegrees);
                return;
            }

            float headingDiff = headingInDegrees - lastPosition.heading;
            float distanceTravelled = Mathf.Abs(driveDiff * physics.distancePerEncoderCountMm / Constants.MM_IN_M);
            int steerDiff = packet.position_steer - lastPacket.position_steer;

            if (Math.Abs(steerDiff) < STEER_CIRCLE_CUTOFF) {
                if(Mathf.Abs(headingDiff) < HEADING_FORWARD_CUTOFF) {
                    EstimatePositionWithLine(distanceTravelled, headingInDegrees, driveDiff);
                } else {
                    EstimatePositionWithCircle(distanceTravelled, headingInDegrees, driveDiff, headingDiff);
                }
                return;
            }

            EstimatePositionWithSteer(distanceTravelled, headingInDegrees, packet.position_steer, driveDiff, headingDiff, steerDiff, timeDiffUs * MS_IN_US);
        }

        private void EstimatePositionWithStop(float headingInDegrees) {
            PublishPacket(lastPosition.position, headingInDegrees);
        }

        private void EstimatePositionWithLine(float distanceTravelled, float headingInDegrees, float driveDiff) {
            float movementDirection = physics.reverseMotorPolarity ^ driveDiff < 0f ? Mathf.PI : 0f;
            var result = Geometry.FromRangeBearing(distanceTravelled, movementDirection, lastPosition);
            PublishPacket(lastPosition.position, headingInDegrees);
        }

        private void EstimatePositionWithCircle(float distanceTravelled, float headingInDegrees, float driveDiff, float headingDiff) {
            //Calculate rotation delta:
            float rotationDeltaMotor = distanceTravelled / physics.turningRadius;
            float rotationDeltaSensor = ((headingInDegrees - lastPosition.heading) * Mathf.PI / 180f);

            if (rotationDeltaSensor > Geometry.HALF_CIRCLE) {
                rotationDeltaSensor = rotationDeltaSensor - Geometry.FULL_CIRCLE;
            } else if (rotationDeltaSensor < -Geometry.HALF_CIRCLE) {
                rotationDeltaSensor = Geometry.FULL_CIRCLE + rotationDeltaSensor;
            }

            if (Mathf.Abs(rotationDeltaMotor - rotationDeltaSensor) > 0.5f) {
                Debug.LogWarning("Ignoring broken measurement! " + rotationDeltaSensor + ", " + rotationDeltaMotor);
                return; //ignore packet
            }

            rotationDeltaMotor /= 2f;//???
            rotationDeltaSensor /= 4f;//???
            if (physics.reverseMotorPolarity ^ driveDiff < 0f) {
                rotationDeltaSensor = Mathf.PI - rotationDeltaSensor;
            }
            //TODO: REWORK TURNING DIAMETER!
            float range = physics.turningDiameter * Mathf.Sin(rotationDeltaMotor);
            var result = Geometry.FromRangeBearing(range, 2f * Mathf.PI - rotationDeltaSensor, lastPosition);

            PublishPacket(result, headingInDegrees);
        }

        private void EstimatePositionWithSteer(float distanceTravelled, float headingInDegrees, int steerPosition, float driveDiff, float headingDiff, float steerDiff, float timeDiff) {
            float estimatedVelocity = distanceTravelled / timeDiff;
            float estimatedSteerSpeed = steerDiff / timeDiff;
            float steerDuration = estimatedSteerSpeed * physics.maxSteerRange;
            float steerBeginOffset = estimatedSteerSpeed * (lastPacket.position_steer - physics.frontSteerPosition);
            //TODO: calculating range and bearing from spiral of steerRadius(x)
            var result = Vector3.zero;
            PublishPacket(result, headingInDegrees);
        }

        private void PublishPacket(Vector3 result, float headingInDegrees) {
            if (float.IsNaN(result.x) || float.IsNaN(result.z) || float.IsNaN(headingInDegrees)) {
                Debug.LogWarning("Ignoring misscalculation");
                return;
            }
            // Finally update the position and heading
            lastPosition.position = result;
            lastPosition.heading = headingInDegrees;
            positionHistory.PutThreadSafe(lastPosition);
        }

	public PositionData TestProcessPacket(CarReconningPacket packet) {
		ProcessPacket(packet);
		return lastPosition;
	}
	

    #region ReplayableUDPServer
    public override string ModuleCall() {
        return "ev3car-reconning " + network.hostIp + " " + moduleNetwork.port + " " + module.pollMs;
    }

    public override int ModulePriority() {
        return module.priority;
    }

    public override bool ModuleAutostart() {
        return module.autostart;
    }

    public override int CreationDelayMs() {
        return module.creationDelayMs;
    }
    #endregion
}
}
