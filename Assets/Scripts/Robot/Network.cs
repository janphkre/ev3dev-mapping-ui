using UnityEngine;

namespace ev3devMapping {

public class Network : MonoBehaviour, ItemCopy<Network> {
	public string hostIp="192.168.0.103";
	public string robotIp="192.168.0.101";

	public Network DeepCopy()
	{
		Network other = (Network) this.MemberwiseClone();
		other.hostIp = string.Copy(hostIp);
		other.robotIp = string.Copy(robotIp);
		return other;
	}

    public void copyFrom(Network other) {
        hostIp = string.Copy(other.hostIp);
        robotIp = string.Copy(other.robotIp);
    }
}
}