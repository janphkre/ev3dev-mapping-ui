using ev3devMapping.Society;
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
    private Vector3 previousPosition = Vector3.zero;
    private float initialHeading;
    private int headingDrift = 0;
    
    protected override void Awake() {
        base.Awake();
        lastPosition = new PositionData { position = transform.parent.position, heading = transform.parent.eulerAngles.y };
        initialHeading = lastPosition.heading;
    }

    protected override void Start() {
        base.Start();
    }
    
    protected void Update() {
        PositionData currentPosition;
        try {
            currentPosition = positionHistory.GetNewestThreadSafe();
        } catch(InvalidOperationException) {
            return;
        }
        transform.parent.transform.position = currentPosition.position;
		transform.parent.transform.rotation = Quaternion.AngleAxis(-currentPosition.heading, Vector3.up);
            if(!currentPosition.position.Equals(previousPosition)) {
                GameObject o = GameObject.CreatePrimitive(PrimitiveType.Cube);
                o.transform.position = currentPosition.position;
                o.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                o.GetComponent<MeshRenderer>().material.color = Color.red;
                previousPosition = currentPosition.position;
            }
        
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
        // Calculate the motor displacement since last packet:
            float driveDiff = packet.position_drive - lastPacket.position_drive;
        
            if (Mathf.Abs(driveDiff) < 0.01f) {
        	//remove drift from heading:
            headingDrift += packet.heading - lastPacket.heading;
        }   
        float headingInDegrees = ((headingDrift - packet.heading) / 100.0f) + initialHeading;
        lastPacket.CloneFrom(packet);
	
        //Calculate rotation delta:
        float delta1 = Mathf.Abs(driveDiff * physics.distancePerEncoderCountMm / Constants.MM_IN_M) / physics.turningRadius;
        float delta2 = ((headingInDegrees - lastPosition.heading) * Mathf.PI / 180f);

		if(delta2 > Geometry.HALF_CIRCLE) {
			delta2 = delta2 - Geometry.FULL_CIRCLE;
		} else if(delta2 < -Geometry.HALF_CIRCLE) {
			delta2 = Geometry.FULL_CIRCLE + delta2;
		}

		if(Mathf.Abs(delta1 - delta2) > 0.5f ) {
            Debug.LogWarning("Ignoring broken measurement! " + delta2 + ", " + delta1);
            return; //ignore packet
        }

		delta1 /= 2f;
		delta2 = delta2 / 4f;
        if (physics.reverseMotorPolarity ^ driveDiff < 0f) {
            delta2 = Mathf.PI - delta2;
        }

        float range = physics.turningDiameter * Mathf.Sin(delta1);
        var result = Society.Geometry.FromRangeBearing(range, 2f*Mathf.PI-delta2, lastPosition);
        
	
		if (float.IsNaN(result.x) || float.IsNaN(result.y) || float.IsNaN(headingInDegrees)) {
            Debug.LogWarning("Ignoring misscalculation");
	    return;
        }
        
        // Finally update the position and heading
        lastPosition.timestamp = packet.timestamp_us;
        lastPosition.position = result;
        lastPosition.heading = headingInDegrees;
        positionHistory.PutThreadSafe(lastPosition);
    }

	public PositionData TestProcessPacket(CarReconningPacket packet) {
		ProcessPacket(packet);
		return lastPosition;
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
