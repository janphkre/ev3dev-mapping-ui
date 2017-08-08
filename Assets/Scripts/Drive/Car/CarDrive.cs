using System;

[Serializable]
public class CarDriveModuleProperties : ModuleProperties {
    public int timeoutMs = 500;
}

//As ReplayableUDPClient is not based on UDPClient we have to use this client although we do not use the replay at the moment.
class CarDrive: ReplayableUDPClient<CarDrivePacket> {

    private static short STEER_FORWARD = 128;
    private static short STEER_BACKWARD = -127;

    public ulong keepAliveUs = 100000L; //100ms * 1000

    private CarDriveModuleProperties module;
    private CarDrivePacket packet = new CarDrivePacket();
    
    protected override void OnDestroy() {
        Halt();
        base.OnDestroy();
    }

    public void Update() {
        if (Timestamp.TimestampUs() - packet.timestamp_us < keepAliveUs) return;
        packet.timestamp_us = Timestamp.TimestampUs();
        packet.command = CarDrivePacket.Commands.KEEPALIVE;
        Send(packet);
    }

    public void Halt() {
        packet.timestamp_us = Timestamp.TimestampUs();
        packet.command = CarDrivePacket.Commands.STOP;
        Send(packet);
    }

    //WaitWhile functionality for SteerForward / SteerBackwards
    public bool IsTurning() {
        if(IsPacketWaiting()) {
            ReceiveOne(packet);
            if(packet.command == CarDrivePacket.Commands.TURNSTOP) return false;
        }
        return true;
    }

    public void Steer(float segment, bool backwards) {
        packet.timestamp_us = Timestamp.TimestampUs();
        packet.command = CarDrivePacket.Commands.TURN;
        packet.param2 = (short) (segment * MainMenu.Physics.DifferentialRadiusMm * MainMenu.Physics.CountsPerMM());
        packet.param1 = backwards ? STEER_BACKWARD : STEER_FORWARD;
        Send(packet);
    }

    //Moves forward for the segment of the max steering angle circle and stops afterwards.
    public void SteerForward(float segment) {
        packet.timestamp_us = Timestamp.TimestampUs();
        packet.command = CarDrivePacket.Commands.TURNSTOP;
        packet.param2 = (short)(segment * MainMenu.Physics.DifferentialRadiusMm * MainMenu.Physics.CountsPerMM());
        packet.param1 = STEER_FORWARD;
        Send(packet);
    }

    //Same as SteerForward but backwards
    public void SteerBackwards(float segment) {
        packet.timestamp_us = Timestamp.TimestampUs();
        packet.command = CarDrivePacket.Commands.TURNSTOP;
        packet.param2 = (short)(segment * MainMenu.Physics.DifferentialRadiusMm * MainMenu.Physics.CountsPerMM());
        packet.param1 = STEER_BACKWARD;
        Send(packet);
    }

#region ReplayableUDPClient
    protected override void Start() { packet.timestamp_us = Timestamp.TimestampUs(); }

    public override string ModuleCall() { return "ev3car-drive " + moduleNetwork.port + " " + module.timeoutMs; }

    public override int ModulePriority() { return module.priority; }

    public override bool ModuleAutostart() { return module.autostart; }

    public override int CreationDelayMs() { return module.creationDelayMs; }
#endregion
}
