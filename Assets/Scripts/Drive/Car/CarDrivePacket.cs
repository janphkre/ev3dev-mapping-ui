using System.Net;

namespace ev3devMapping {

/* * * * * * * * * * * * * * * * * * * * * *
 * See Assets/Scripts/Drive/DrivePacket.cs *
 * * * * * * * * * * * * * * * * * * * * * */
class CarDrivePacket : IDatagram {

    public enum Commands : short { KEEPALIVE = 0, TURN = 1, FORWARD = 2, BACKWARD = 3, STOP = 4 , TURNSTOP = 5 };

    public ulong timestamp_us;
    public Commands command;
    public short param1;//forward/backward
    public short param2;//steering segment

    public override string ToString() {
        return string.Format("[ts={0} cmd={1} a1={2} a2={3}]", timestamp_us, command, param1, param2);
    }

    public void FromBinary(System.IO.BinaryReader reader) {
        timestamp_us = (ulong)IPAddress.NetworkToHostOrder(reader.ReadInt64());
        command = (Commands)IPAddress.NetworkToHostOrder(reader.ReadInt16());
        param1 = IPAddress.NetworkToHostOrder(reader.ReadInt16());
        param2 = IPAddress.NetworkToHostOrder(reader.ReadInt16());
    }

    public void ToBinary(System.IO.BinaryWriter writer) {
        writer.Write(IPAddress.HostToNetworkOrder((long)timestamp_us));
        writer.Write(IPAddress.HostToNetworkOrder((short)command));
        writer.Write(IPAddress.HostToNetworkOrder(param1));
        writer.Write(IPAddress.HostToNetworkOrder(param2));
    }

    public int BinarySize() {
        return 14; //8 + 3*2 = 14 bytes
    }

    public ulong GetTimestampUs() {
        return timestamp_us;
    }
}
}
