using UnityEngine;

namespace ev3devMapping {

public enum TachometerPosition { Differential, Left, Right }

public class Physics : MonoBehaviour, ItemCopy<Physics>
{
    public float turningRadius = 1.19f;
	public float wheelDiameterMm = 92.8f;
	public float wheelbaseMm = 185.0f;
	public int encoderCountsPerRotation=360;
	public int maxEncoderCountsPerSecond=1000;
	public bool reverseMotorPolarity=false;
    public TachometerPosition Differential = TachometerPosition.Right;

    //Calculated at start:
    public float turningRadiusSquared;
    public float turningRadiusAngledSquared;
    public float turningDiameter;
    public float halfWheelbase;
    public float differentialRadiusCounts;
    public float distancePerEncoderCountMm;

	public float CountsPerMM()
	{
		return encoderCountsPerRotation / (Mathf.PI * wheelDiameterMm);
	}
	
    public void Calculate() {
        turningDiameter = 2f * turningRadius;
        turningRadiusSquared = turningRadius * turningRadius;
        turningRadiusAngledSquared = Mathf.Sqrt(2f * turningRadiusSquared);
        halfWheelbase = wheelbaseMm / 2000.0f;
        /* 
         * As the distance on the segment of a circle equals to radius * angle,
         * we multiply the segment angle with either the turningRadius or the outerTurningRadius (if the car does not have a differential).
         * As the robot measures the car movement in tachometer counts we multiply the above with counts per meter.
         */
        differentialRadiusCounts = 1000.0f * CountsPerMM();
        distancePerEncoderCountMm = Mathf.PI * wheelDiameterMm / encoderCountsPerRotation;
    }

	public Physics DeepCopy()
	{
		Physics other = (Physics) this.MemberwiseClone();
		return other;
	}

    public void copyFrom(Physics other) {
        turningRadius = other.turningRadius;
        wheelDiameterMm = other.wheelDiameterMm;
        wheelbaseMm = other.wheelbaseMm;
        encoderCountsPerRotation = other.encoderCountsPerRotation;
        maxEncoderCountsPerSecond = other.maxEncoderCountsPerSecond;
        reverseMotorPolarity = other.reverseMotorPolarity;
        Differential = other.Differential;
        Calculate();
    }
}
}