using ev3devMapping.Society;
using UnityEngine;

namespace ev3devMapping.Testing {

	//This script is attached to the "Settings" GameObject.
	public class TestingReconning : ITesting {

		private const float STEP_SIZE = 10f;
		private const int MAX_HEADING = 18000;
		private CarReconning reconning = null;
		private CarReconningPacket testPacket = new CarReconningPacket();
		
		private LineRenderer rendererX;
		private LineRenderer rendererY;
		
		public void Test(GameObject robot) {
			MainMenu.Physics = new Physics();
		    MainMenu.Physics.turningRadius = 0.595f;
		    MainMenu.Physics.wheelbaseMm = 185.0f;
		    MainMenu.Physics.Calculate();
		   	
		   	GameObject obj = new GameObject("RendererX");
			rendererX = obj.AddComponent<LineRenderer>();
			rendererX.startWidth = 0.01f;
			rendererX.endWidth = 0.01f;
			rendererX.positionCount = 0;
			obj = new GameObject("RendererY");
			rendererY = obj.AddComponent<LineRenderer>();
			rendererY.startWidth = 0.01f;
			rendererY.endWidth = 0.01f;
			rendererY.positionCount = 0;	
		   	
		    reconning = robot.transform.Find("CarReconning").GetComponent<CarReconning>();
		}
	
		void Update () {
			if(reconning != null) {
				testPacket.timestamp_us = Timestamp.TimestampUs();
				Debug.Log("Testing Packet " + testPacket.position_drive + ", " + testPacket.heading);
				var position = reconning.TestProcessPacket(testPacket);
				testPacket.position_drive += (int) (STEP_SIZE / MainMenu.Physics.distancePerEncoderCountMm);
				testPacket.heading += (short) (STEP_SIZE * 18f / (Mathf.PI * MainMenu.Physics.turningRadius));
				if(testPacket.heading > MAX_HEADING) {
					testPacket.heading = (short) ((int) testPacket.heading - 2 * MAX_HEADING);
				} else if (testPacket.heading < -MAX_HEADING) {
					testPacket.heading = (short) ((int) testPacket.heading +  2 * MAX_HEADING);				
				}
				var lastPose = new Vector3(position.position.x,position.position.z, position.heading);
				var positiveTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, Geometry.RIGHT_ANGLE, lastPose);
        		var negativeTurningCenter = Geometry.FromRangeBearing(MainMenu.Physics.turningRadius, -Geometry.RIGHT_ANGLE, lastPose);
				Planing.DrawArc(rendererX, 0.2f, positiveTurningCenter, -Geometry.RIGHT_ANGLE + lastPose.z, Geometry.HALF_CIRCLE);
				Planing.DrawArc(rendererX, 0.2f, negativeTurningCenter, Geometry.RIGHT_ANGLE + lastPose.z, -Geometry.HALF_CIRCLE);
			}
		}
	}

}
