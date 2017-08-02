using UnityEngine;

public class Physics : MonoBehaviour, ItemCopy<Physics>
{
    public float innerTurningDiameter = 0.0f;
    public float turningRadius = 0.0f;
    public float turningRadiusSquared = 0.0f;
    public float turningRadiusAngledSquared = 0.0f;//Math.Sqrt(2*turningRadiusSquared)
    public float halfWheelbase = 0.0f;
    public float maxTurningAngle = 0.0f;//TODO!
	public float wheelDiameterMm=43.2f;
	public float wheelbaseMm=250.0f;
	public int encoderCountsPerRotation=360;
	public int maxEncoderCountsPerSecond=1000;
	public bool reverseMotorPolarity=false;
    

    public double TurningRadius { get; internal set; }

    public float MMPerCount()
	{
		return Mathf.PI * wheelDiameterMm / encoderCountsPerRotation;
	}
	public float CountsPerMM()
	{
		return encoderCountsPerRotation / (Mathf.PI * wheelDiameterMm);
	}
		
	public Physics DeepCopy()
	{
		Physics other = (Physics) this.MemberwiseClone();
		return other;
	}

    public void copyFrom(Physics other) {
        wheelDiameterMm = other.wheelDiameterMm;
        wheelbaseMm = other.wheelbaseMm;
        encoderCountsPerRotation = other.encoderCountsPerRotation;
        maxEncoderCountsPerSecond = other.maxEncoderCountsPerSecond;
        reverseMotorPolarity = other.reverseMotorPolarity;
    }
}
