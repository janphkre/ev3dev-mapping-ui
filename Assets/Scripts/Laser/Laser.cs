﻿/*
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

using UnityEngine;
using System;
using System.IO;
using ev3devMapping.Society;

namespace ev3devMapping {

public enum PlotType {Local, Global, Map, GlobalWithMap}

[Serializable]
public class LaserModuleProperties : ModuleProperties
{
	public string laserDevice = "/dev/tty_in1";
	public string motorPort = "outC";
	public int laserDutyCycle = 44;
	public int crcTolerancePct = 10;
    public float laserOffset = -90f;
}
	
[Serializable]
public class LaserPlotProperties
{
	public PlotType plotType;
	public float distanceLimit=10.0f;
    public float minimumDistanceMm = 0.1f;
	public PointCloud laserPointCloud;
}
	
class LaserThreadSharedData
{
    public const int READINGS_LENGTH = 360;
	public Vector3[] readings=new Vector3[READINGS_LENGTH];
	public bool[] invalid_data = new bool[READINGS_LENGTH];
	public int from = -1;
	public int length = -1;
	public bool consumed = true;
	public float averagedPacketTimeMs;
	public float laserRPM;
	public int crcFailurePercentage=0;
	public int invalidPercentage=0;

	public void CopyNewDataFrom(LaserThreadSharedData other)
	{
		// Copy only the data that changed since last call
		from = other.from;
		length = other.length;
		averagedPacketTimeMs = other.averagedPacketTimeMs;
		laserRPM = other.laserRPM;
		crcFailurePercentage = other.crcFailurePercentage;
		invalidPercentage = other.invalidPercentage;

		for (int i = from; i < from + length; ++i)
		{
			int ind = i % readings.Length;
			readings[ind] = other.readings[ind];
			invalid_data[ind] = other.invalid_data[ind];
		}
	}
}

class LaserThreadInternalData
{
	public Vector3[] readings = new Vector3[LaserThreadSharedData.READINGS_LENGTH];
	public bool[] invalid_data = new bool[LaserThreadSharedData.READINGS_LENGTH];
	public ulong[] timestamps = new ulong[LaserThreadSharedData.READINGS_LENGTH];

	public bool pending=false;
	public int pending_from=0;
	public int pending_length=0;
	public ulong t_from=0;
	public ulong t_to=0;
	public float laserRPM;
	public int invalidCount=0;
	public int invalidPercentage = 0;
	public int crcFailures=0;
	public int crcFailurePercentage=0;
    public int packageCounter = 0;

	public void SetPending(int from, int length, ulong time_from, ulong time_to)
	{
		pending = true;
		pending_from = from;
		pending_length = length;
		t_from = time_from;
		t_to = time_to;
	}
}

[RequireComponent (typeof (LaserUI))]
//[RequireComponent (typeof (Map3D))]
public class Laser : ReplayableUDPServer<LaserPacket>
{
	public const ushort LIDAR_CRC_FAILURE_ERROR_CODE = 0x66;

	public LaserModuleProperties module = null;
	public LaserPlotProperties plot = null;

	private PointCloud laserPointCloud;
	private Map3D map3D = null;

	private LaserThreadSharedData data=new LaserThreadSharedData();

	private Matrix4x4 laserTRS;

	#region UDP Thread Only Data
	private LaserThreadInternalData threadInternal = new LaserThreadInternalData ();
    private PositionData lastPackageLastPos;

	#endregion

	#region Thread Shared Data
	private LaserThreadSharedData threadShared = new LaserThreadSharedData();
	#endregion
			
	protected override void OnDestroy()
	{
		base.OnDestroy ();
	}

	protected override void Awake()
	{
		base.Awake();
		laserPointCloud = SafeInstantiate<PointCloud> (plot.laserPointCloud);
		laserTRS =  Matrix4x4.TRS (transform.localPosition, transform.localRotation, Vector3.one);
	}

	protected override void Start ()
	{
		//map3D = GetComponent<Map3D> (); //No map for you!
		base.Start();
	}

	void Update ()
	{
		lock (threadShared)
		{
			if (threadShared.consumed)
				return; //no new data, nothing to do
			data.CopyNewDataFrom(threadShared);
			threadShared.consumed = true;
		}
			
		if (data.length > 360)
			print ("Huh, does this ever happen? If so we can optimize");

		if(plot.plotType!=PlotType.Map)
			laserPointCloud.SetVertices(data.readings);

		if(map3D!=null && plot.plotType==PlotType.Map || plot.plotType==PlotType.GlobalWithMap)
			map3D.AssignVertices (data.readings, data.from, data.length, data.invalid_data);

	}

	#region UDP Thread Only Functions

	protected override void ProcessPacket(LaserPacket packet)
	{			
		threadInternal.laserRPM = packet.laser_speed / 64.0f;

		// if we had unprocessed packet last time do it now
		if (plot.plotType != PlotType.Local && threadInternal.pending)
		{
			int i_from = threadInternal.pending_from, len=threadInternal.pending_length;
			ulong t_from = threadInternal.t_from, t_to = threadInternal.t_to;
			if (TranslateReadingsToGlobalReferenceFrame (i_from, len, t_from, t_to))
				PushCalculatedReadingsThreadSafe (i_from, len);
		}

            int processedReadings= CalculateReadingsInLocalReferenceFrame(packet);

		if (plot.plotType != PlotType.Local)
            if (!TranslateReadingsToGlobalReferenceFrame (packet.laser_angle, processedReadings, packet.timestamp_us, packet.GetEndTimestampUs()))
				return; //don't use the readings yet (or at all), no position data in this timeframe

		PushCalculatedReadingsThreadSafe (packet.laser_angle, processedReadings);
	}

	private int CalculateReadingsInLocalReferenceFrame(LaserPacket packet)
	{
		int angle_index;
		float alpha, distance_mm, angle;

		Vector3[] readings = threadInternal.readings;
		bool[] invalid_data = threadInternal.invalid_data;
		ulong[] timestamps = threadInternal.timestamps;

		for (int i = 0; i < packet.laser_readings.Length; ++i)		
		{
			angle_index = packet.laser_angle + i;
			angle = angle_index;

			if (angle_index == 0)
			{
				threadInternal.crcFailurePercentage = threadInternal.crcFailures * 100 / 360;
				threadInternal.invalidPercentage = threadInternal.invalidCount * 100 / 360;
				threadInternal.crcFailures = threadInternal.invalidCount = 0;
            } else if(angle_index >= readings.Length) {
                return i;
            }

			readings[angle_index] = Vector3.zero;
			timestamps[angle_index] = packet.GetTimestampUs(i);
			invalid_data[angle_index] = packet.laser_readings[i].invalid_data == 1;
				
			//if distance is greater than maximum we allow, mark reading as inalid
			invalid_data[angle_index] |= packet.laser_readings[i].distance > plot.distanceLimit * 1000;
            //invalid_data[angle_index] |= packet.laser_readings[i].distance < plot.minimumDistanceMm;

            if (invalid_data[angle_index]) {
                ++threadInternal.invalidCount;
                if (packet.laser_readings[i].distance == LIDAR_CRC_FAILURE_ERROR_CODE)
                    ++threadInternal.crcFailures;
                continue;
            }
			// calculate reading in laser plane
			distance_mm = packet.laser_readings[i].distance;
			alpha = angle + module.laserOffset;
            Vector3 pos = new Vector3(-(distance_mm * Mathf.Sin(alpha * Constants.DEG2RAD)) / 1000.0f, 0f,  (distance_mm * Mathf.Cos(alpha * Constants.DEG2RAD)) / 1000.0f);
			// translate/rotate reading taking into acount laser mounting position and rotation
			readings[angle_index] = laserTRS.MultiplyPoint3x4 (pos);
		}
            return packet.laser_readings.Length;
	}

	private bool TranslateReadingsToGlobalReferenceFrame(int from, int len, ulong t_from, ulong t_to)
	{
		bool not_in_history, not_yet;
		PositionHistory.PositionSnapshot snapshot= positionHistory.GetPositionSnapshotThreadSafe(t_from, t_to, out not_yet, out not_in_history);
		threadInternal.pending = false;

		if (not_in_history)
		{
			print("laser - ignoring packet (position data not in history) with timestamp " + t_from);
			return false;
		}
		if (not_yet)
		{
			threadInternal.SetPending (from, len, t_from, t_to);
			return false;
		}
			
		Matrix4x4 robotToGlobal = new Matrix4x4();
		Vector3 scale = Vector3.one;
		PositionData pos=new PositionData();
		ulong[] timestamps = threadInternal.timestamps;
		Vector3[] readings = threadInternal.readings;

        ulong firstTimestamp = ulong.MaxValue;
        ulong lastTimestamp = ulong.MinValue;

        for (int i = from; i < from+len; ++i)
		{
			pos = snapshot.PositionAt(timestamps[i]);
			robotToGlobal.SetTRS(pos.position, Quaternion.AngleAxis(-pos.heading, Vector3.up), scale);
			readings[i]=robotToGlobal.MultiplyPoint3x4(readings[i]);
            //readings[i].z = 1;
            //Save first and last position for this package:
            if (timestamps[i] < firstTimestamp) {
                firstTimestamp = timestamps[i];
            }
            if (timestamps[i] > lastTimestamp) {
                lastTimestamp = timestamps[i];
                lastPackageLastPos = pos;
            }
        }

		return true;
	}

	public void PushCalculatedReadingsThreadSafe(int from, int length)
	{
		lock (threadShared)
		{
			Array.Copy(threadInternal.readings, from, threadShared.readings, from, length);
			Array.Copy(threadInternal.invalid_data, from, threadShared.invalid_data, from, length);

			if (threadShared.consumed)
			{
				threadShared.from = from;
				threadShared.length = length;
			}
			else //data was not consumed yet
				threadShared.length += length;

			threadShared.consumed = false;
			threadShared.averagedPacketTimeMs = AveragedPacketTimeMs();
			threadShared.laserRPM = threadInternal.laserRPM;
			threadShared.invalidPercentage = threadInternal.invalidPercentage;
			threadShared.crcFailurePercentage = threadInternal.crcFailurePercentage;
		}

        threadInternal.packageCounter += length;
        if (threadInternal.packageCounter >= LaserThreadSharedData.READINGS_LENGTH) {
            threadInternal.packageCounter = 0;
            if (threadInternal.invalidCount >= LaserThreadSharedData.READINGS_LENGTH) {
                Debug.LogError("The whole array is invalid!");
                threadInternal.invalidCount = 0;
                return;
            }
            //SLAMRobot.singelton.PostOdometryAndReadings(new SLAMInputData(lastPackageLastPos, threadInternal.readings, from, length, threadInternal.invalid_data, threadInternal.invalidPackageCount));
            Planing.singleton.LaserReadings = new PlaningInputData(threadInternal.readings, threadInternal.invalid_data, threadInternal.invalidCount);
            threadInternal.invalidCount = 0;
        }
    }


	#endregion

	#region UI reactions

	public void SaveMap()
	{
		if (map3D == null)
		{
			print("map is null, unable to save!");
			return;
		}

		string robotName = transform.parent.name;

		Directory.CreateDirectory(Config.MapPath(robot.sessionDirectory, robotName));

		print("saving map to file \"" + Config.MapPath(robot.sessionDirectory, robotName, name) + "\"");

		map3D.SaveToPlyPolygonFileFormat(Config.MapPath(robot.sessionDirectory, robotName, name), "created with ev3dev-mapping");
	}

	#endregion

	public float GetAveragedPacketTimeMs()
	{
		return data.averagedPacketTimeMs;
	}
	public float GetAveragedLaserRPM()
	{
		return data.laserRPM;
	}
	public float GetInvalidPercentage()
	{
		return data.invalidPercentage;
	}

	public float GetCRCFailurePercentage()
	{
		return data.crcFailurePercentage;
	}

	#region RobotModule

	public override string ModuleCall()
	{
		return "ev3laser " + module.laserDevice + " " + module.motorPort + " " + network.hostIp + " " + moduleNetwork.port + " " + module.laserDutyCycle + " " + module.crcTolerancePct;
	}
	public override int ModulePriority()
	{
		return module.priority;
	}
	public override bool ModuleAutostart()
	{
		return module.autostart;
	}
	public override int CreationDelayMs()
	{
		return module.creationDelayMs;
	}

	#endregion

}
}
