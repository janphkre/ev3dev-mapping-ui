using System;
using UnityEngine;

namespace ev3devMapping {

[Serializable]
public class CarReconningModuleProperties: ModuleProperties {
    public int pollMs = 10;
}

class CarReconning: ReplayableUDPServer<CarReconningPacket> {

    public CarReconningModuleProperties module = null;
    
    private CarReconningPacket lastPacket = new CarReconningPacket();
    private PositionData lastPosition = new PositionData();
    private float initialHeading;

    protected override void Awake() {
        base.Awake();
        lastPosition = new PositionData { position = transform.parent.position, heading = transform.parent.eulerAngles.y };
        initialHeading = lastPosition.heading;
    }

    protected override void Start() {
        base.Start();
    }

    protected override void ProcessPacket(CarReconningPacket packet) {
        //First call - set first udp packet with reference encoder positions
        if (lastPacket.timestamp_us == 0) {
            lastPacket.CloneFrom(packet);
            return;
        }

        // UDP doesn't guarantee ordering of packets, if previous odometry is newer ignore the received
        if (packet.timestamp_us <= lastPacket.timestamp_us) {
            print("car-reconning - ignoring out of time packet (previous, now):" + Environment.NewLine + lastPacket.ToString() + Environment.NewLine + packet.ToString());
            return;
        }

        // Calculate the linear displacement since last packet:
        float ddiff = packet.position_drive - lastPacket.position_drive;
        float distanceTravelled = Mathf.Abs(ddiff * physics.distancePerEncoderCountMm / Constants.MM_IN_M);
        
        //Calculate rotation delta:
        float delta = packet.HeadingInDegrees - lastPacket.HeadingInDegrees;
        delta = (delta * Mathf.PI) / 180f;
        if (physics.reverseMotorPolarity ^ ddiff < 0f) delta = Mathf.PI - delta;
        delta = delta % Mathf.PI;
        float range = physics.turningDiameter * Mathf.Sin(distanceTravelled / physics.turningDiameter);
        var result = Society.Geometry.FromRangeBearing(range, ((lastPacket.HeadingInDegrees * Mathf.PI) / 180f) + delta);
        // Finally update the position and heading
        lastPosition.timestamp = packet.timestamp_us;
        lastPosition.position = new Vector3(lastPosition.position.x + result.x, lastPosition.position.y, lastPosition.position.z + result.y);
        lastPosition.heading = packet.HeadingInDegrees + initialHeading;

        lastPacket.CloneFrom(packet);
        if(float.IsNaN(lastPosition.position.x) || float.IsNaN(lastPosition.position.y) || float.IsNaN(lastPosition.position.z) || float.IsNaN(lastPosition.heading))
                throw new ArgumentException();
        positionHistory.PutThreadSafe(lastPosition);
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