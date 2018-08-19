using System;

namespace ev3devMapping {

[Serializable]
public class CarDriveModuleProperties: ModuleProperties {
    public int timeoutMs = 500;
}

//As ReplayableUDPClient is not based on UDPClient we have to use this client although we do not use the replay at the moment.
class CarDrive: ReplayableUDPClient<CarDrivePacket> {

    private static short STEER_FORWARD = 128;
    private static short STEER_BACKWARD = -127;

    public static ulong KEEP_ALIVE_US = 200 * 1000; //200 (ms) * 1000 (us)
    public CarDriveModuleProperties module = null;
    
    private ulong lastTimestamp;

    protected override void OnDestroy() {
        Halt();
        base.OnDestroy();
    }

    protected override void Start() {
        lastTimestamp = Timestamp.TimestampUs();
    }

    public void Update() {
        if ((Timestamp.TimestampUs() - lastTimestamp) < KEEP_ALIVE_US) return;
        lastTimestamp = Timestamp.TimestampUs();
        CarDrivePacket packet = new CarDrivePacket();
        packet.timestamp_us = lastTimestamp;
        packet.command = CarDrivePacket.Commands.KEEPALIVE;
        Send(packet);
    }

    public void Halt() {
        lastTimestamp = Timestamp.TimestampUs();
        CarDrivePacket packet = new CarDrivePacket();
        packet.timestamp_us = lastTimestamp;
        packet.command = CarDrivePacket.Commands.STOP;
        Send(packet);
    }

    public void Steer(float segment, bool backwards) {
        lastTimestamp = Timestamp.TimestampUs();
        CarDrivePacket packet = new CarDrivePacket();
        packet.timestamp_us = lastTimestamp;
        packet.command = CarDrivePacket.Commands.TURN;
        packet.param2 = CalculateSegment(segment);
        packet.param1 = backwards != physics.reverseMotorPolarity ? STEER_BACKWARD : STEER_FORWARD;
        Send(packet);
    }

    //Moves forward for the segment of the max steering angle circle and stops afterwards.
    public void SteerForward(float segment) {
        lastTimestamp = Timestamp.TimestampUs();
        CarDrivePacket packet = new CarDrivePacket();
        packet.timestamp_us = lastTimestamp;
        packet.command = CarDrivePacket.Commands.TURNSTOP;
        packet.param2 = CalculateSegment(segment);
            packet.param1 = physics.reverseMotorPolarity ? STEER_BACKWARD : STEER_FORWARD;
        Send(packet);
    }

    //Same as SteerForward but backwards
    public void SteerBackwards(float segment) {
        lastTimestamp = Timestamp.TimestampUs();
        CarDrivePacket packet = new CarDrivePacket();
        packet.timestamp_us = lastTimestamp;
        packet.command = CarDrivePacket.Commands.TURNSTOP;
        packet.param2 = CalculateSegment(segment);
            packet.param1 = physics.reverseMotorPolarity ? STEER_FORWARD : STEER_BACKWARD;
        Send(packet);
    }
    
    public void DriveAhead(bool backwards) {
        lastTimestamp = Timestamp.TimestampUs();
        CarDrivePacket packet = new CarDrivePacket();
        packet.timestamp_us = lastTimestamp;
            packet.command = backwards != physics.reverseMotorPolarity ? CarDrivePacket.Commands.BACKWARD : CarDrivePacket.Commands.FORWARD;
        Send(packet);
    }
    
    private short CalculateSegment(float segment) {
        if (MainMenu.Physics.Differential == TachometerPosition.Differential) {
            segment *= MainMenu.Physics.turningRadius;
        } else if (MainMenu.Physics.Differential == TachometerPosition.Left ^ segment > 0) {
            segment *= MainMenu.Physics.turningRadius + MainMenu.Physics.halfWheelbase;
        } else {
            segment *= MainMenu.Physics.turningRadius - MainMenu.Physics.halfWheelbase;
        }
        return (short)(segment * MainMenu.Physics.differentialRadiusCounts);
    }

#region ReplayableUDPClient

    public override string ModuleCall() { return "ev3car-drive " + moduleNetwork.port + " " + module.timeoutMs; }

    public override int ModulePriority() { return module.priority; }

    public override bool ModuleAutostart() { return module.autostart; }

    public override int CreationDelayMs() { return module.creationDelayMs; }
#endregion
}
}
