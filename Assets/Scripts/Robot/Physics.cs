using UnityEngine;

public class Physics : MonoBehaviour, ItemCopy<Physics>
{
    public float turningRadius = 0.0f;
	public float wheelDiameterMm=43.2f;
	public float wheelbaseMm=250.0f;
	public int encoderCountsPerRotation=360;
	public int maxEncoderCountsPerSecond=1000;
	public bool reverseMotorPolarity=false;
    public bool hasDifferential = false;
    //Calculated at start:
    public float turningRadiusSquared;
    public float turningRadiusAngledSquared;
    public float innerTurningDiameter;
    public float halfWheelbase;
    public float differentialRadiusCounts;
    public float distancePerEncoderCountMm;

	public float CountsPerMM()
	{
		return encoderCountsPerRotation / (Mathf.PI * wheelDiameterMm);
	}
	
    public void Calculate() {
        innerTurningDiameter = 2f * turningRadius - (wheelbaseMm / 1000.0f);
        turningRadiusSquared = turningRadius * turningRadius;
        turningRadiusAngledSquared = Mathf.Sqrt(2f * turningRadiusSquared);
        halfWheelbase = wheelbaseMm / 2000.0f;
        /* 
         * As the distance on the segment of a circle equals to radius * angle,
         * we multiply the segment angle with either the turningRadius or the outerTurningRadius (if the car does not have a differential).
         * As the robot measures the car movement in tachometer counts we multiply the above with counts per meter.
         */
        differentialRadiusCounts = (hasDifferential ? turningRadius : turningRadius + halfWheelbase) * 1000.0f * CountsPerMM();
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
        hasDifferential = other.hasDifferential;
        Calculate();
    }
}
