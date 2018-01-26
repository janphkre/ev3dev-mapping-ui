using UnityEngine;
using System.Collections;

namespace ev3devMapping {

public class Limits : MonoBehaviour, ItemCopy<Limits>
{
	public float MaxLinearSpeedMmPerSec=200;
	public float MaxAngularSpeedDegPerSec=45;

	public float MaxAngularSpeedRadPerS()
	{
		return Mathf.Deg2Rad * MaxAngularSpeedDegPerSec;
	}
		
	public Limits DeepCopy()
	{
		Limits other = (Limits) this.MemberwiseClone();
		return other;
	}

    public void copyFrom(Limits other) {
        MaxLinearSpeedMmPerSec = other.MaxLinearSpeedMmPerSec;
        MaxAngularSpeedDegPerSec = other.MaxAngularSpeedDegPerSec;
    }

}
}