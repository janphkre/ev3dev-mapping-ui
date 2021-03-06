﻿using System.Net;

namespace ev3devMapping {

class CarReconningPacket : IDatagram {

    public ulong timestamp_us = 0;
    public int position_drive;
    public int position_steer;
    public short heading;

    public float HeadingInDegrees {
        get { return heading / 100.0f; }
    }

    public override string ToString() {
            return string.Format("[timestamp={0} pd={1} ps={2} h={3}]", timestamp_us, position_drive, position_steer, heading);
    }

    public void FromBinary(System.IO.BinaryReader reader) {
        timestamp_us = (ulong)IPAddress.NetworkToHostOrder(reader.ReadInt64());
        position_drive = IPAddress.NetworkToHostOrder(reader.ReadInt32());
        position_steer = IPAddress.NetworkToHostOrder(reader.ReadInt32());
        heading = IPAddress.NetworkToHostOrder(reader.ReadInt16());
    }
    public void ToBinary(System.IO.BinaryWriter writer) {
        writer.Write(IPAddress.HostToNetworkOrder((long)timestamp_us));
        writer.Write(IPAddress.HostToNetworkOrder(position_drive));
        writer.Write(IPAddress.HostToNetworkOrder(position_steer));
        writer.Write(IPAddress.HostToNetworkOrder(heading));
    }

    public int BinarySize() {
        return 18; // 8 + 4 + 4 + 2
    }

    public void CloneFrom(CarReconningPacket p) {
        timestamp_us = p.timestamp_us;
        position_drive = p.position_drive;
        position_steer = p.position_steer;
        heading = p.heading;
    }

    public ulong GetTimestampUs() {
        return timestamp_us;
    }
}
}
