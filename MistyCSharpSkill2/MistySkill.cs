using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MistyRobotics.Common;
using MistyRobotics.SDK.Commands;
using MistyRobotics.Common.Data;
using MistyRobotics.SDK.Events;
using MistyRobotics.SDK.Responses;
using MistyRobotics.Common.Types;
using MistyRobotics.SDK;
using MistyRobotics.SDK.Messengers;
using System.Drawing;
using System.Drawing.Imaging;
using Windows.Graphics.Imaging;
using Windows.Storage.Streams;
using Windows.Storage;
using System.Timers;
using System.Collections.ObjectModel;

namespace MistyMapSkill2
{
	internal class MistySkill : IMistySkill
    {
		/// <summary>
		/// Make a local variable to hold the misty robot interface, call it whatever you want 
		/// </summary>
		private IRobotMessenger _misty;

		/// <summary>
		/// Stores data from the Inertial Measurement Unity such as pitch, roll, yaw
		/// </summary>
		private IIMUEvent IMUData;

		/// <summary>
		/// You can get data such as distance drive for each track and current velocity of misty's tracks
		/// </summary>
		private IDriveEncoderEvent driveEncoderData;

		/// <summary>
		/// Data type that stores the name of a certain Time of Flight sensor (frontRight, frontCenter, etc...)
		/// </summary>
		private TimeOfFlightPosition tofPosition;

		/// <summary>
		/// This is the variable that stores the slam map that is currently being created in this program
		/// </summary>
		MistyRobotics.Common.Data.SlamMap map = new SlamMap();

		/// <summary>
		/// Set to true if misty has received data from the IMU, which also means that IMUData will be initialized
		/// </summary>
		private bool IMUEventReceived;

		/// <summary>
		/// Holds the current pose of misty and is updated when a SelfStateEvent is received and processed
		/// </summary>
		private RobotPose pose = new RobotPose();

		/// <summary>
		/// We assume that the user will place misty in the center of the room at the start of the program, 
		/// so we will use the initial coordinates as a marker which misty will return to and venture to and from
		/// </summary>
		private RobotPose initialPose = new RobotPose();

		/// <summary>
		/// This is used as a signal to prevent misty from receiving multiple "findPose()" commands at once from multiple events which may call "findPose()"
		/// </summary>
		///private DriveEncoderEvent 

		/// <summary>
		/// This is used as a signal to prevent misty from receiving movement commands when she is already moving from a hazard.
		/// Set to true at the start of a hazard processing and false at the end of a hazard processing.
		/// </summary>
		private bool isMovingFromHazard = false;

		/// <summary>
		/// This is used as a signal to prevent misty from receiving multiple "findPose()" commands at once from multiple events which may call "findPose()"
		/// </summary>
		bool currentlyFindingPose = false;

		/// <summary>
		/// SLAM status: does misty know her position in her environment?
		/// </summary>
		bool hasPose = false;

		/// <summary>
		/// SLAM status: does misty know her position in her environment?
		/// </summary>
		bool lostPose = false;

		/// <summary>
		/// SLAM status: misty is currently exploring
		/// </summary>
		bool isExploring = false;

		/// <summary>
		/// SLAM status: misty is Slam Streaming (SLAM is active but may not be mapping, may be tracking)
		/// </summary>
		bool isStreaming = false;

		/// <summary>
		/// SLAM status: misty has tracking active
		/// </summary>
		bool isTracking = false;

		/// <summary>
		/// Increment this on timer callback
		/// </summary>
		private static int elapsedSeconds = 0;

		/// <summary>
		/// Used to check how long misty has been stationary during "dumbRoaming()"
		/// </summary>
		private static int secondsSinceCommandCalled = 0;

		/// <summary>
		/// Ideally used to keep track of what misty is currently doing during "dumbRoaming()", but was replaced by "robotState"
		/// </summary>
		private enum DumbRoamStates { Drive360, DriveStraight, NA };
		DumbRoamStates roamStates = DumbRoamStates.NA;

		/// <summary>
		/// Used to keep track of what misty is currently doing
		/// </summary>
		private enum RobotState { Spinning, DriveStraight, DriveBackwards, Stopped, Hazard, NA};
		/// <summary>
		/// Used to keep track of what misty is currently doing
		/// </summary>
		private RobotState robotState;

		/// <summary>
		/// Potentially should just be a boolean, but is used to tell if misty is currently in the middle of the "SpinTillOpenArea()" function. 
		/// This was necessary because often times hazards would call this function many times, so this prevents the function from being called like 10 times in a row.
		/// </summary>
		private enum IsSpinning { NotSpinning, Spinning};

		/// <summary>
		/// Potentially should just be a boolean, but is used to tell if misty is currently in the middle of the "SpinTillOpenArea()" function. 
		/// This was necessary because often times hazards would call this function many times, so this prevents the function from being called like 10 times in a row.
		/// </summary>
		private IsSpinning isSpinning;
		
		private enum mappingPhase { Initializing, DumbRoam, SemiSmartRoam, SmartRoam };

		/// <summary>
		/// This is set by the "closestTOFReading()" function. It represents the closest object to any of the 3 front tof sensors.
		/// </summary>
		private double closestObject = 0;

		/// <summary>
		/// Current distance reading of the frontRight time of flight sensor
		/// </summary>
		private double frontRightTOF = 0;

		/// <summary>
		/// Current distance reading of the frontLeft time of flight sensor
		/// </summary>
		private double frontLeftTOF = 0;

		/// <summary>
		/// Current distance reading of the frontCenter time of flight sensor
		/// </summary>
		private double frontCenterTOF = 0;
		
		/// <summary>
		/// Skill details for the robot
		/// 
		/// There are other parameters you can set if you want:
		///   Description - a description of your skill
		///   TimeoutInSeconds - timeout of skill in seconds
		///   StartupRules - a list of options to indicate if a skill should start immediately upon startup
		///   BroadcastMode - different modes can be set to share different levels of information from the robot using the 'SkillData' websocket
		///   AllowedCleanupTimeInMs - How long to wait after calling OnCancel before denying messages from the skill and performing final cleanup  
		/// </summary>
		public INativeRobotSkill Skill { get; private set; } = new NativeRobotSkill("MistyMapSkill2", "93e9b284-b07d-443d-8339-64f96b483b07");

		/// <summary>
		///	This method is called by the wrapper to set your robot interface
		///	You need to save this off in the local variable commented on above as you are going use it to call the robot
		/// </summary>
		/// <param name="robotInterface"></param>
		public void LoadRobotConnection(IRobotMessenger robotInterface)
		{
			_misty = robotInterface;

		}



		/// <summary>
		/// This event handler is called when the robot/user sends a start message
		/// The parameters can be set in the Skill Runner (or as json) and used in the skill if desired
		/// </summary>
		/// <param name="parameters"></param>
		public async void OnStart(object sender, IDictionary<string, object> parameters)
		{
			

			// simply change color to show that the skill has begun
			_misty.ChangeLED(0, 200, 0, LEDResponse);

			// Listen to the IMU event which gives positional data about misty
			_misty.RegisterIMUEvent(IMUCallback, 0, true, null, null, null);

			// Listen to the drive encoder event which we use for its velocity information (check if misty is moving or not)
			// Add a function (+=) that gets called whenever the event is received
			_misty.RegisterDriveEncoderEvent(100, true, null, "Drive Encoder Event", OnResponse);
			_misty.DriveEncoderEventReceived += ProcessDriveEncoderEvent;

			// Listen to the self state event which we use to get misty's pose, or position in the world
			// once again, add a function (+=) that gets called whenever the event is received/triggered
			_misty.RegisterSelfStateEvent(250, true, "SelfState", OnResponse);
			_misty.SelfStateEventReceived += ProcessSelfStateEvent;

			// Listen for hazard events. By default most hazards just cause misty to stop moving. We would like misty to navigate away from hazards
			// once again, add a function (+=) that gets called whenever the event is received/triggered
			_misty.RegisterHazardNotificationEvent(300, true, "Hazard Event", OnResponse);
			_misty.HazardNotificationEventReceived += ProcessHazardEvent;

			// Listen to the slam status event, set booleans such as "isExploring", "hasPose", "isStreaming" to true or false
			_misty.RegisterSlamStatusEvent(20, true, "Slam Status", null, null);
			_misty.SlamStatusEventReceived += ProcessSlamStatusEvent;

			// Check description (All functions should have descriptions if you hover over them)
			registerTOFEvents();


			// =============================================================================================================
			// Temporary code, isolating certain parts of the code to test more efficiently
			// =============================================================================================================



			/*
			IGetCurrentSlamMapResponse currentMap = await _misty.GetCurrentSlamMapAsync();

			Debug.WriteLine("current map: " + currentMap.Data);
			await _misty.StartSlamStreamingAsync();
			await _misty.StartTrackingAsync();

			while (!(isTracking && hasPose && isStreaming)) 
			{
				IGetSlamStatusResponse slamStatus = await _misty.GetSlamStatusAsync();

				//SlamStatusDetails slamStatusDetails = slamStatus.Data;

				//for (int i = 0; i < slamStatusDetails.StatusList.Length; i++)
				//{
				//	//Debug.WriteLine("Slam status[" + i + "]: " + slamStatusDetails.StatusList[i]);
				//	if (slamStatusDetails.StatusList[i] == "Exploring")
				//		isExploring = true;
				//	else if (slamStatusDetails.StatusList[i] == "HasPose")
				//		hasPose = true;
				//	else if (slamStatusDetails.StatusList[i] == "Streaming")
				//		isStreaming = true;
				//	else if (slamStatusDetails.StatusList[i] == "LostPose")
				//		hasPose = false;
				//	else if (slamStatusDetails.StatusList[i] == "Tracking")
				//		isTracking = true;
				//}
			}

			initialPose = pose;
			await semiSmartRoam();
	



			//await spinTillOpenArea(1);
			//dumbRoaming();
			await _misty.StartMappingAsync();

			await _misty.StartTrackingAsync();
		
			while (true)
				{
				IGetSlamStatusResponse slamStatus = await _misty.GetSlamStatusAsync();
			
                for (int i = 0; i < slamStatus.Data.StatusList.Length; i++)
                {
                    Debug.WriteLine("Slam status[" + i + "]: " + slamStatus.Data.StatusList[i]);
                    if (slamStatus.Data.StatusList[i] == "Exploring")
                        isExploring = true;
                    else if (slamStatus.Data.StatusList[i] == "HasPose")
                        hasPose = true;
                    else if (slamStatus.Data.StatusList[i] == "Streaming")
                        isStreaming = true;
                    else if (slamStatus.Data.StatusList[i] == "LostPose")
                        hasPose = false;
                    else if (slamStatus.Data.StatusList[i] == "Tracking")
                        isTracking = true;
                }
				await Task.Delay(100);
            }

					*/

			//	_misty.RegisterObstacleMapEvent(300, true, "Obstacle Event", null);
			//_misty.ObstacleMapEventReceived += ProcessObstacleMapEvent;

			// =============================================================================================================
			// \/ Real code begins again here \/ 
			// =============================================================================================================


			// keep attempting to initialize the slam mapping until misty has a pose (and is exploring and streaming, but those are usually not the problem
			do
			{
				Debug.WriteLine("Starting mapping (again)");
				await initializeMapping();
			} while (!(isExploring && hasPose && isStreaming));

			// again, ensure that misty has a pose
			if (isExploring && hasPose && isStreaming)
			{
				
				IGetMapResponse mapResponse;

				// keep driving in circles till the map has some sort of content
				do
				{
					Debug.WriteLine("Trying to get initial map (size > 0)");
					await drive360();
					mapResponse = await _misty.GetMapAsync();
					map = mapResponse.Data;
					Debug.WriteLine("Map size: " + map.Size);
					Debug.WriteLine("Map width: " + map.Width);

				} while (map.Size == 0);

				initialPose = pose;
				Debug.WriteLine("initial pose: " + initialPose);

				// not sure if I should be turning on tracking or not
				//await _misty.StartTrackingAsync();
				dumbRoaming();

				await _misty.StartTrackingAsync();
				await semiSmartRoam();
                
				// I think this is unneccesary now
				int width = map.Width;
				int height = map.Height;

				var bitMap = setByteArray();
				setBitmap(bitMap);


				// just make sure to stop mapping and streaming when the program ends or else misty will just keeping going
				await _misty.StopMappingAsync();
				await _misty.StopSlamStreamingAsync();

			}


			// change the leds to show that the program is over
			_misty.ChangeLED(0, 0, 200, LEDResponse);

			// Cancel the skill
			_misty.CancelRunningSkill("93e9b284-b07d-443d-8339-64f96b483b07", null);

		}



		/// <summary>
		/// Simply update the pose variable to make it current.
		/// </summary>
		private void ProcessSelfStateEvent(object sender, ISelfStateEvent selfStateEvent)
		{
			pose = selfStateEvent.Pose;
			//pose.;
		}

		/// <summary>
		/// Set isExploring, hasPose, isStreaming, and isTracking to reflect misty's current slam status.
		/// If misty does not have a pose yet, find it.
		/// Consider changing this from booleans to some other data type that could combine 4 variables into 1.
		/// </summary>
		private async void ProcessSlamStatusEvent(object sender, ISlamStatusEvent slamStatusEvent) 
		{
			//SlamStatusEvent statusEvent = (SlamStatusEvent)slamStatusEvent;
			
			for (int i = 0; i < slamStatusEvent.SlamStatus.StatusList.Length; i++)
			{
				Debug.WriteLine("Slam status[" + i + "]: " + slamStatusEvent.SlamStatus.StatusList[i]);
				if (slamStatusEvent.SlamStatus.StatusList[i] == "Exploring")
					isExploring = true;
				else if (slamStatusEvent.SlamStatus.StatusList[i] == "HasPose")
				{
					hasPose = true;
					lostPose = false;
				}
				else if (slamStatusEvent.SlamStatus.StatusList[i] == "Streaming")
					isStreaming = true;
				else if (slamStatusEvent.SlamStatus.StatusList[i] == "Tracking")
					isTracking = false;
				else if (slamStatusEvent.SlamStatus.StatusList[i] == "LostPose")
				{
					hasPose = false;
					lostPose = true;
				}

			}

			if(lostPose == true && currentlyFindingPose == false)
            {
				// still very much working on this part so it is commented out for now
				await findPose();
            }
		}

		/// <summary>
		/// Simply store the drive encoder data in the "driveEncoderData" variable. Will be used to check if misty is moving or stationary.
		/// </summary>
		private void ProcessDriveEncoderEvent(object sender, IDriveEncoderEvent driveEncoderEvent)
		{
			driveEncoderData = driveEncoderEvent;

			
			
		}

		/// <summary>
		/// A somewhat complicated-to-write callback function that attempts to move misty away from any hazards
		/// </summary>
		private void ProcessHazardEvent(object sender, IHazardNotificationEvent hazardEvent)
		{
			
			//_misty.get hazardEvent.Created;
			//if (!isMovingFromHazard)
			//{
				// set isMovingFromHazard to true so that certain outside functions will not interfere with the hazard processing
				isMovingFromHazard = true;

				// turns out that "CurrentSensorHazard" does NOT mean the sensor that is currently in hazard, but a sensor that has a electrical current related problem :eyeroll:
				Collection<SensorHazardStatus> hazardList = new Collection<SensorHazardStatus>(hazardEvent.CurrentSensorsHazard);

				// list of sensors that have caused misty to stop moving
				IList<SensorHazardStatus> driveStoppedList = hazardEvent.DriveStopped;

				// this is mostly for debugging, just to know what is triggering the hazard event
				getHazardType(hazardEvent);

				/*
				Debug.WriteLine("HazardList: ");
				for (int i = 0; i < hazardList.Count(); i++) {
					Debug.WriteLine(hazardList[i].SensorName + ": " + hazardList[i].InHazard);

				}
				Debug.WriteLine("DriveStoppedList: ");
				for (int i = 0; i < driveStoppedList.Count(); i++)
				{
					Debug.WriteLine(driveStoppedList[i].SensorName + ": " + hazardList[i].InHazard);

				}
				*/

				// can't drive forward or back, spin in place
				if (hazardEvent.FrontHazard && hazardEvent.BackHazard)
				{
					_misty.DriveArc(IMUData.Yaw - 35, 0, 2500, false, DriveArcResponse);
					System.Threading.Thread.Sleep(2500);
					//await Task.Delay(2500);
					Debug.WriteLine("Hazard processing: front and back");
				}
				// can't drive forward, so back up and turn away
				else if (hazardEvent.FrontHazard)
				{
					Debug.WriteLine("Attempting to drive backwards");
					_misty.DriveTime(-10, 0, 4000, null);
					
					System.Threading.Thread.Sleep(4000);
					//await Task.Delay(4000);
					_misty.DriveArc(IMUData.Yaw - 25, 0, 2500, false, null);
					System.Threading.Thread.Sleep(2500);
					//await Task.Delay(2500);
					Debug.WriteLine("2.5 seconds later: turning 25 degrees");

					//await spinTillOpenArea(1.25);
					//_misty.DriveArc(IMUData.Yaw - 25, 0, 2500, false, OnResponse);
					//System.Threading.Thread.Sleep(2500);

				}
				// can't drive backwards, so go forward
				else if (hazardEvent.BackHazard)
				{
					/*
					_misty.DriveTime(10, 0, 2500, HazardCallback);
					_misty.PlayAudio("001-EeeeeeE.wav", 1, PlayAudioResponse);
					System.Threading.Thread.Sleep(2500);
					_misty.DriveArc(IMUData.Yaw - 25, 0, 2500, false, DriveArcResponse);
					System.Threading.Thread.Sleep(2500);
					*/
					Debug.WriteLine("Attempting to drive forward");
					_misty.DriveTime(10, 0, 4000, null);
					//_misty.PlayAudio("001-EeeeeeE.wav", 1, PlayAudioResponse);
					System.Threading.Thread.Sleep(4000);
					//await Task.Delay(4000);
					Debug.WriteLine("4 seconds later: turning 25 degrees");
					_misty.DriveArc(IMUData.Yaw - 25, 0, 2500, false, null);
					System.Threading.Thread.Sleep(2500);
					//await Task.Delay(2500);
					
					//await spinTillOpenArea(1.25);
				}
				else
				{
					//await spinTillOpenArea();
				}
				isMovingFromHazard = false;
			//}
			//else
			//{
			//	//Debug.WriteLine("Oof, hazard got denied");

			//}

		}

		/// <summary>
		/// Used for debugging, to know what type of hazard triggered the hazard event
		/// </summary>
		private void getHazardType(IHazardNotificationEvent hazardEvent)
		{
			if (hazardEvent.ArmCurrentHazard)
			{
				Debug.WriteLine("Hazard processing: arm current hazard");
			}
			else if (hazardEvent.ArmSpeedHazard)
			{
				Debug.WriteLine("Hazard processing: arm speed hazard");
			}
			else if (hazardEvent.ArmStallHazard)
			{
				Debug.WriteLine("Hazard processing: arm stall hazard");
			}
			else if (hazardEvent.BackHazard)
			{
				Debug.WriteLine("Hazard processing: back hazard");
				for (int i = 0; i < hazardEvent.DriveStopped.Count; i++)
				{
					if (hazardEvent.DriveStopped.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.DriveStopped.ElementAt(i).SensorName + ": " + hazardEvent.DriveStopped.ElementAt(i).InHazard);

				}
				for (int i = 0; i < hazardEvent.CurrentSensorsHazard.Count; i++)
				{
					if (hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.CurrentSensorsHazard.ElementAt(i).SensorName + ": " + hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard);

				}

			}
			else if (hazardEvent.HeadCurrentHazard)
			{
				Debug.WriteLine("Hazard processing: HeadCurrentHazard");
			}
			else if (hazardEvent.FrontHazard)
			{
				Debug.WriteLine("Hazard processing: FrontHazard");

				for (int i = 0; i < hazardEvent.DriveStopped.Count; i++)
				{
					if (hazardEvent.DriveStopped.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.DriveStopped.ElementAt(i).SensorName + ": " + hazardEvent.DriveStopped.ElementAt(i).InHazard);

				}
				for (int i = 0; i < hazardEvent.CurrentSensorsHazard.Count; i++)
				{
					if (hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.CurrentSensorsHazard.ElementAt(i).SensorName + ": " + hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard);

				}
			}
			else if (hazardEvent.HeadSpeedHazard)
			{
				Debug.WriteLine("Hazard processing: HeadSpeedHazard");
			}
			else if (hazardEvent.HeadStallHazard)
			{
				Debug.WriteLine("Hazard processing: headstallHazard");
			}
			else if (hazardEvent.StopHazard)
			{
				Debug.WriteLine("Hazard processing: StopHazard");
				for (int i = 0; i < hazardEvent.DriveStopped.Count; i++)
				{
					if (hazardEvent.DriveStopped.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.DriveStopped.ElementAt(i).SensorName + ": " + hazardEvent.DriveStopped.ElementAt(i).InHazard);

				}
				for (int i = 0; i < hazardEvent.CurrentSensorsHazard.Count; i++)
				{
					if (hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.CurrentSensorsHazard.ElementAt(i).SensorName + ": " + hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard);

				}
			}
			else if (hazardEvent.TrackCurrentHazard)
			{
				Debug.WriteLine("Hazard processing: TrackCurrentHazard");
			}
			else if (hazardEvent.TrackSpeedHazard)
			{
				Debug.WriteLine("Hazard processing: TrackSpeedHazard");
			}
			else if (hazardEvent.TrackStallHazard)
			{
				Debug.WriteLine("Hazard processing: TrackStallHazard");
			}
			else
			{
				Debug.WriteLine("Hazard processing: none of the boolean hazard types");
				for (int i = 0; i < hazardEvent.DriveStopped.Count; i++)
				{
					if(hazardEvent.DriveStopped.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.DriveStopped.ElementAt(i).SensorName + ": " + hazardEvent.DriveStopped.ElementAt(i).InHazard);
					
				}
				for (int i = 0; i < hazardEvent.CurrentSensorsHazard.Count; i++)
				{
					if (hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard)
						Debug.WriteLine(hazardEvent.CurrentSensorsHazard.ElementAt(i).SensorName + ": " + hazardEvent.CurrentSensorsHazard.ElementAt(i).InHazard);

				}
			}
		}

		/// <summary>
		/// 
		/// </summary>
		private void ProcessTimeOfFlightEvent(object sender, ITimeOfFlightEvent TimeOfFlightEvent)
		{
			//Debug.WriteLine("Processing tof event");
			if (TimeOfFlightEvent.SensorPosition == TimeOfFlightPosition.FrontCenter) // && TimeOfFlightEvent.Status == 0
			{
				frontCenterTOF = TimeOfFlightEvent.DistanceInMeters;
				//Debug.WriteLine("Front Center updooted: " + frontCenterTOF);
			}
			else if (TimeOfFlightEvent.SensorPosition == TimeOfFlightPosition.FrontRight)  // && TimeOfFlightEvent.Status == 0
			{
				frontRightTOF = TimeOfFlightEvent.DistanceInMeters;
				//Debug.WriteLine("Front Right updoote: " + frontRightTOF);
			}
			else if (TimeOfFlightEvent.SensorPosition == TimeOfFlightPosition.FrontLeft) // && TimeOfFlightEvent.Status == 0
			{
				frontLeftTOF = TimeOfFlightEvent.DistanceInMeters;
				//Debug.WriteLine("Front Left updooted: " + frontLeftTOF);
				//Debug.WriteLine(TimeOfFlightEvent.SensorPosition + ": " + closestObject);
			}
			else if (TimeOfFlightEvent.Status == 104 || TimeOfFlightEvent.Status == 102)
			{
				switch (TimeOfFlightEvent.SensorPosition)
				{
					case TimeOfFlightPosition.FrontLeft:
						Debug.WriteLine("Front Left updooted: " + frontLeftTOF);
						frontLeftTOF = 2;
						break;
					case TimeOfFlightPosition.FrontRight:
						Debug.WriteLine("Front Right updoote: " + frontRightTOF);
						frontRightTOF = 2;
						break;
					case TimeOfFlightPosition.FrontCenter:
						Debug.WriteLine("Front Center updooted: " + frontCenterTOF);
						frontCenterTOF = 2;
						break;
				}
			}
		}

		/// <summary>
		/// This sets the value of closestObject to the front sensor with the closest distance reading.
		/// </summary>
		private void closestTOFSensorReading()
		{
			if (frontCenterTOF <= frontRightTOF && frontCenterTOF <= frontLeftTOF)
			{
				closestObject = frontCenterTOF;
				tofPosition = TimeOfFlightPosition.FrontCenter;
			}
			else if (frontRightTOF <= frontCenterTOF && frontRightTOF <= frontLeftTOF)
			{
				closestObject = frontRightTOF;
				tofPosition = TimeOfFlightPosition.FrontRight;
			}
			else if (frontLeftTOF <= frontRightTOF && frontLeftTOF <= frontCenterTOF)
			{
				tofPosition = TimeOfFlightPosition.FrontLeft;
				closestObject = frontLeftTOF;
			}
		}

		/// <summary>
		/// Keep spinning till there are no objects within a certain distance being reading by the tof sensors
		/// </summary>
		private void spinTillOpenArea(double distance)
		{
			Debug.WriteLine("Starting Spinntillopenarea()");
			int msElapsed;
			int degreesTurned;
			isSpinning = IsSpinning.Spinning;

			 //_misty.Stop(null);

			do
			{
				degreesTurned = 0;
				msElapsed = 0;
				TimeOfFlightPosition initialClosestSensor = TimeOfFlightPosition.Unknown;
				while (!IMUEventReceived)
				{
					Debug.WriteLine(IMUEventReceived);
					System.Threading.Thread.Sleep(1000);
				}


				closestTOFSensorReading();
				//IRobotCommandResponse driveArcResponse;
				int i = 0;
				while (i < 4 && closestObject < distance)
				{
					msElapsed = 0;


					// basically this code is checking the initial direction to turn then its gonna keep turning in that direction till it finds open area
					if (i == 0 || initialClosestSensor == TimeOfFlightPosition.Unknown)
					{
						initialClosestSensor = turnDirection();
					}
					else if (initialClosestSensor == TimeOfFlightPosition.FrontLeft)
					{
						_misty.DriveArc(IMUData.Yaw - 90, 0, 7500, false, null);
					}
					else if (initialClosestSensor == TimeOfFlightPosition.FrontRight)
					{
						_misty.DriveArc(IMUData.Yaw + 90, 0, 7500, false, null);
					}


					Debug.WriteLine("Closest Object: " + closestObject);
					Debug.WriteLine("frontlefttof: " + frontLeftTOF);
					Debug.WriteLine("frontRighttof: " + frontRightTOF);
					Debug.WriteLine("frontCentertof: " + frontCenterTOF);
					Debug.WriteLine("");

					do
					{

						closestTOFSensorReading();
						//Debug.WriteLine("Drive arc response: " + driveArcResponse.Status);

						degreesTurned = degreesTurned + 5;
						msElapsed = msElapsed + 100;
						System.Threading.Thread.Sleep(100);
						//await Task.Delay(100);

					} while (closestObject < distance && msElapsed <= 7500); //&& degreesTurned < 355  && driveArcResponse.Status == ResponseStatus.Success
					i++;
				}
				//_misty.Stop(null);

				if (msElapsed >= 29000 || closestObject < distance) //degreesTurned >= 355
				{
					degreesTurned = 0;
					_misty.DriveTime(-10, 0, 2500, null);
					//await Task.Delay(2500);
					System.Threading.Thread.Sleep(2500);
					//await spinTillOpenArea(distance);
				}
				_misty.PlayAudio("001-EeeeeeE.wav", 1, null);


				
			} while (msElapsed >= 29000 || closestObject < distance);
		
			Debug.WriteLine("Open area to drive was found!");
			isSpinning = IsSpinning.NotSpinning;
		}

		/// <summary>
		/// Turn in a specific direction based on which sensor has the object closest to it
		/// </summary>
		private TimeOfFlightPosition turnDirection()
        {
			switch (tofPosition) 
			{
				case TimeOfFlightPosition.FrontCenter:
				case TimeOfFlightPosition.FrontLeft:
					_misty.DriveArc(IMUData.Yaw - 90, 0, 7500, false, null);
					return TimeOfFlightPosition.FrontLeft;
					break;
				case TimeOfFlightPosition.FrontRight:
					_misty.DriveArc(IMUData.Yaw + 90, 0, 7500, false, null);
					return TimeOfFlightPosition.FrontRight;
					break;

			};
			return TimeOfFlightPosition.Unknown;
        }

		/// <summary>
		/// Register only certain TOF sensors so that we dont waste too much resources
		/// </summary>
		private void registerTOFEvents()
		{
			//Register Bump Sensors with a callback
			//_misty.RegisterBumpSensorEvent(BumpCallback, 50, true, null, null, null);

			//Front Right Time of Flight
			List<TimeOfFlightValidation> tofFrontRightValidations = new List<TimeOfFlightValidation>();
			tofFrontRightValidations.Add(new TimeOfFlightValidation { Name = TimeOfFlightFilter.SensorName, Comparison = ComparisonOperator.Equal, ComparisonValue = TimeOfFlightPosition.FrontRight });
			_misty.RegisterTimeOfFlightEvent(10, true, tofFrontRightValidations, "FrontRight", null);


			//Front Left Time of Flight
			List<TimeOfFlightValidation> tofFrontLeftValidations = new List<TimeOfFlightValidation>();
			tofFrontLeftValidations.Add(new TimeOfFlightValidation { Name = TimeOfFlightFilter.SensorName, Comparison = ComparisonOperator.Equal, ComparisonValue = TimeOfFlightPosition.FrontLeft });
			_misty.RegisterTimeOfFlightEvent(10, true, tofFrontLeftValidations, "FrontLeft", null);

			//Front Center Time of Flight
			List<TimeOfFlightValidation> tofFrontCenterValidations = new List<TimeOfFlightValidation>();
			tofFrontCenterValidations.Add(new TimeOfFlightValidation { Name = TimeOfFlightFilter.SensorName, Comparison = ComparisonOperator.Equal, ComparisonValue = TimeOfFlightPosition.FrontCenter });
			_misty.RegisterTimeOfFlightEvent(10, true, tofFrontCenterValidations, "FrontCenter", null);



			_misty.TimeOfFlightEventReceived += ProcessTimeOfFlightEvent;


		}

		/// <summary>
		/// Very new function meant to spin till misty finds a pose
		/// </summary>
		private async Task findPose()
        {
			currentlyFindingPose = true;
			IRobotCommandResponse poseEstimationResponse = new RobotCommandResponse();
			do
			{
				poseEstimationResponse = await _misty.StartPoseEstimationAsync(.1, 0);
				Debug.WriteLine("PoseEstimationResponse: " + poseEstimationResponse.Status);
			} while (poseEstimationResponse.Status != ResponseStatus.Success);

			_misty.DriveAsync(-5, 0);
			while(closestObject < 1)
            {
				closestTOFSensorReading();
            }
			while (!hasPose)
			{ 

				await _misty.DriveArcAsync(IMUData.Yaw - 10, .05, 2500, false);

				//System.Threading.Thread.Sleep(15000);
				await Task.Delay(5000);
			}


			currentlyFindingPose = false;
		}

		/// <summary>
		/// Keep restarting mapping till this finds a pose
		/// </summary>
		private async Task initializeMapping()
		{

			await _misty.StopMappingAsync();
			await _misty.StopSlamStreamingAsync();

			await _misty.StartMappingAsync();
			_misty.StartObstacleDetection(300, null);
			await _misty.DriveArcAsync(IMUData.Yaw - 25, .05, 2500, false);

			//System.Threading.Thread.Sleep(15000);
			await Task.Delay(15000);
			


			IGetSlamStatusResponse slamStatus = await _misty.GetSlamStatusAsync();
			SlamStatusDetails slamStatusDetails = slamStatus.Data;

			for (int i = 0; i < slamStatusDetails.StatusList.Length; i++)
			{
				Debug.WriteLine("Slam status[" + i + "]: " + slamStatusDetails.StatusList[i]);
				if (slamStatusDetails.StatusList[i] == "Exploring")
					isExploring = true;
				else if (slamStatusDetails.StatusList[i] == "HasPose")
					hasPose = true;
				else if (slamStatusDetails.StatusList[i] == "Streaming")
					isStreaming = true;
			}

		}

		/// <summary>
		/// This sets the value of IMUData to the current IMUEvent data. I should probably get rid of the boolean.
		/// </summary>
		private void IMUCallback(IIMUEvent data)
		{
			IMUData = data;
			IMUEventReceived = true;
		}


		/// <summary>
		/// Not at all done, but will try to use the map created in dumb roam to explore out in a direction, then use tracking to return to the original position.
		/// Potentially relies too much on misty making a good map nad not running into hazards
		/// </summary>
		private async Task semiSmartRoam()
		{
			Debug.WriteLine("SemiSmart Roam");
			double x = initialPose.X;
			double y = initialPose.Y;
			Debug.WriteLine("Target pose: " + x + ":" + y);
			Debug.WriteLine("Pre path pose: " + pose.X + ":" + pose.Y);
			bool pathCompleted = false;
			double turnDegrees = 20;
			double degreesTurned = 0;
			//string coordinates = (x.ToString() + ":" + y.ToString());
			string coordinates = "";

			while(degreesTurned <= 360)
			{
				await _misty.DriveArcAsync(IMUData.Yaw - turnDegrees, 0, 2500, false);
				degreesTurned += turnDegrees;
				await Task.Delay(2500);
				await driveThereAndBack((int)x, (int)y, coordinates);
			}
			//Debug.WriteLine("Semi Smart Roam started");
			//Debug.WriteLine("OriginX: " + map.OriginX);
			//Debug.WriteLine("OriginY: " + map.OriginY);
			//Debug.WriteLine("Pose: " + pose.X + ":" + pose.Y);

			//IRobotCommandResponse pathResponse = await _misty.FollowPathAsync("0:0", 10, null, 0, null);
			//await Task.Delay(10000);
			//Debug.WriteLine("Path error msg: " + pathResponse.ErrorMessage);
			//Debug.WriteLine("Path following status: " + pathResponse.Status);
			//await drive360();
			//Debug.WriteLine("Pose after following the path: " + pose.X + ":" + pose.Y);

		}

		/// <summary>
		/// Drive in a direction till you find an object, then return to the original center position
		/// </summary>
		private async Task<bool> driveThereAndBack(int x, int y, string coordinates)
        {
			await _misty.DriveAsync(15, 0);
			do
			{
				 closestTOFSensorReading();

			} while (closestObject > .75);
			Debug.WriteLine("Pre path pose: " + pose.X + ":" + pose.Y);

			await _misty.StopAsync();
			await Task.Delay(2500);

			IGetSlamPathResponse pathResponse = await _misty.GetSlamPathAsync(x, y, .25, .1, true);
			Debug.WriteLine("pathresponse.data.count: " + pathResponse.Data.Count);
			Debug.WriteLine("pathresponse.status: " + pathResponse.Status);

			for (int i = 0; i < pathResponse.Data.Count; i++)
			{
				coordinates = coordinates + pathResponse.Data.ElementAt(i).X + ":" + pathResponse.Data.ElementAt(i).Y + ",";

			}
			coordinates = coordinates.TrimEnd(',');
			Debug.WriteLine("coordinates: " + coordinates);
			IRobotCommandResponse followPatchResponse = await _misty.FollowPathAsync(coordinates, null, null, 0, null);

			Debug.WriteLine("Follow path response status: " + followPatchResponse.Status);
			while((pose.X > initialPose.X - 1 && pose.X < initialPose.X + 1) && (pose.Y > initialPose.Y - 1 && pose.Y < initialPose.Y + 1)) 
			{
				await Task.Delay(100);
			}
			Debug.WriteLine("Post path pose: " + pose.X + ":" + pose.Y);

			if (followPatchResponse.Status == ResponseStatus.Fail || followPatchResponse.Status == ResponseStatus.Ignored 
				|| followPatchResponse.Status == ResponseStatus.Timeout)
            {
				return false;
            }
            else
            {
				return true;

			}
			
		}

		/// <summary>
		/// Increment elapsed seconds and secondsSinceCommandCalled each time the timer ticks
		/// </summary>
		private static void OnTimedEvent(object source, ElapsedEventArgs e)
		{
			elapsedSeconds++;
			secondsSinceCommandCalled++;

		}

		/// <summary>
		/// Basically, roams around semi randomly and tries to stay .5 meters away from all objects in its path.
		/// I set it to .75 because the area I am in is too small to stay a full meter away from all objects
		/// </summary>
		private void dumbRoaming()
		{
			var random = new Random();
			
			//double angularVelocity = 20 * random.NextDouble();
			Debug.WriteLine("Dumb Roaming started");
			System.Timers.Timer timer = new Timer(1000);
			
			timer.Elapsed += new ElapsedEventHandler(OnTimedEvent);
			timer.Start();
			while (elapsedSeconds < 120) // !isMapMostlyFilled(map) && 
			{
				closestTOFSensorReading();
				if (!isMovingFromHazard && closestObject > .75 && robotState != RobotState.DriveStraight)
				{
					double angularVelocity = random.Next(-10, 10);
					Debug.WriteLine("drivestraight (dumb roam)");
					_misty.Drive(10, angularVelocity, DriveResponse);
					//roamStates = DumbRoamStates.DriveStraight;
					robotState = RobotState.DriveStraight;
				
					secondsSinceCommandCalled = 0;

				
				}
				else if (!isMovingFromHazard && closestObject < .75)
				{
					Debug.WriteLine("find open direction (dumb roam)");
					//_misty.Stop(null);
					spinTillOpenArea(1);
					//roamStates = DumbRoamStates.Drive360;
					robotState = RobotState.Spinning;
					
					secondsSinceCommandCalled = 0;

					

					_misty.Drive(10, 0, null);
				}
				else if (isMovingFromHazard)
				{
					Debug.WriteLine("IsMovingFromHazard (dumb roam)");
					//roamStates = DumbRoamStates.NA;
					
					secondsSinceCommandCalled = 0;
				}
				
				// possibly just make this an else statement? Idrk, probably could just merge with the above state
				else if(driveEncoderData.LeftVelocity == 0 && driveEncoderData.RightVelocity == 0
						&& !isMovingFromHazard && robotState != RobotState.Hazard && secondsSinceCommandCalled > 5)
                {
					// I think this is where the problem is, but it has happened in other areas as well. 
					robotState = RobotState.NA;
					secondsSinceCommandCalled = 0;
					Debug.WriteLine("Dumb Roam is not being blocked by hazards, but is neither in state 'drive360' or 'drivestraight'. Try driving backwards.");
					_misty.DriveTime(-10, 0, 2500, null);
					System.Threading.Thread.Sleep(2500);
					//await Task.Delay(2500);
					spinTillOpenArea(1);

					//roamStates = DumbRoamStates.NA;
					
                }
                else
                {
					//Debug.WriteLine("Shrug");
                }
				
			}
			elapsedSeconds = 0;
		}




		/// <summary>
		/// This turns the map into an image that is stored in the same location as the skill.
		/// You can use scp -r administrator@<ip_of_misty>:Address that was output by "Debug.WriteLine("File path: " + outputFile.Path);" to copy all of the maps created by this program
		/// </summary>
		private async void setBitmap(byte[] imageByteData)
		{
			

			string fileName = ("map-" + DateTime.UtcNow + ".png");
			fileName = fileName.Replace(" ", "-");
			fileName = fileName.Replace("/", "-");
			fileName = fileName.Replace(":", "-");

			Debug.WriteLine(fileName);
			Windows.Storage.StorageFolder storageFolder =
				Windows.Storage.ApplicationData.Current.LocalFolder;

			Debug.WriteLine(storageFolder.ToString());
			Windows.Storage.StorageFile outputFile =
				await storageFolder.CreateFileAsync(fileName, Windows.Storage.CreationCollisionOption.ReplaceExisting);

			Debug.WriteLine("File path: " + outputFile.Path);

			SoftwareBitmap softwareBitmap = new SoftwareBitmap(BitmapPixelFormat.Rgba8, map.Width, map.Height);

			using (IRandomAccessStream stream = await outputFile.OpenAsync(FileAccessMode.ReadWrite))
			{
				// Create an encoder with the desired format
				BitmapEncoder encoder = await BitmapEncoder.CreateAsync(BitmapEncoder.PngEncoderId, stream);

				// Set the software bitmap
				encoder.SetSoftwareBitmap(softwareBitmap);
				// Set additional encoding parameters, if needed
				encoder.SetPixelData(BitmapPixelFormat.Rgba8, BitmapAlphaMode.Ignore, Convert.ToUInt32(map.Width), Convert.ToUInt32(map.Height), 600, 600, imageByteData);
				await encoder.FlushAsync();
			}
		}

		/// <summary>
		/// This turns the grid data into pixel data
		/// </summary>
		private byte[] setByteArray()
		{
			int pixelIter = 0;
			byte[] bitMapStream = new byte[4 * map.Size];
			for (int i = 0; i < map.Size; i++)
			{

				if (map.Grid[i] == MapCellState.Unknown)
				{
					bitMapStream[pixelIter++] = 168;
					bitMapStream[pixelIter++] = 168;
					bitMapStream[pixelIter++] = 168;
					bitMapStream[pixelIter++] = 1;
				}
				//Debug.Write(" 0 ");
				else if (map.Grid[i] == MapCellState.Open)
				{
					bitMapStream[pixelIter++] = 255;
					bitMapStream[pixelIter++] = 255;
					bitMapStream[pixelIter++] = 255;
					bitMapStream[pixelIter++] = 1;
					//Debug.Write(" 1 ");
				}
				else if (map.Grid[i] == MapCellState.Occupied)
				{
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 1;
					//Debug.Write(" 2 ");
				}
				else if (map.Grid[i] == MapCellState.Covered)
				{
					bitMapStream[pixelIter++] = 142;
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 152;
					bitMapStream[pixelIter++] = 1;
					//Debug.Write(" 3 ");
				}

				else if (map.Grid[i] == MapCellState.DoesNotExist)
				{
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 0;
					bitMapStream[pixelIter++] = 1;
					//Debug.Write(" D ");
				}

			

			}
			

			return bitMapStream;
		}
		
	
		/// <summary>
		/// Drive in a circle
		/// </summary>
		private async Task drive360()
		{

			while (!IMUEventReceived) { }
			await _misty.DriveArcAsync(IMUData.Yaw - 180, .1, 7000, false);

			_misty.PlayAudio("001-EeeeeeE.wav", 1, PlayAudioResponse); 
			await Task.Delay(7000);
			await _misty.DriveArcAsync(IMUData.Yaw - 180, .1, 7000, false);
			_misty.PlayAudio("001-EeeeeeE.wav", 1, PlayAudioResponse);
			await Task.Delay(7000);

		}

	

		/// <summary>
		/// This event handler is called when Pause is called on the skill
		/// User can save the skill status/data to be retrieved when Resume is called
		/// Infrastructure to help support this still under development, but users can implement this themselves as needed for now 
		/// </summary>
		/// <param name="parameters"></param>
		public void OnPause(object sender, IDictionary<string, object> parameters)
		{
			//In this template, Pause is not implemented by default
		}

		/// <summary>
		/// This event handler is called when Resume is called on the skill
		/// User can restore any skill status/data and continue from Paused location
		/// Infrastructure to help support this still under development, but users can implement this themselves as needed for now 
		/// </summary>
		/// <param name="parameters"></param>
		public void OnResume(object sender, IDictionary<string, object> parameters)
		{
			//TODO Put your code here and update the summary above
		}

		/// <summary>
		/// This event handler is called when the cancel command is issued from the robot/user
		/// You currently have a few seconds to do cleanup and robot resets before the skill is shut down... 
		/// Events will be unregistered for you 
		/// </summary>
		public async void OnCancel(object sender, IDictionary<string, object> parameters)
		{
			//TODO Put your code here and update the summary above
			Debug.WriteLine("OnCancel is being called");

			if (map.IsValid)
			{
				int width = map.Width;
				int height = map.Height;

				var bitMap = setByteArray();
				setBitmap(bitMap);
			}

			await _misty.StopMappingAsync();
			await _misty.StopSlamStreamingAsync();
		}

		/// <summary>
		/// This event handler is called when the skill timeouts
		/// You currently have a few seconds to do cleanup and robot resets before the skill is shut down... 
		/// Events will be unregistered for you 
		/// </summary>
		public void OnTimeout(object sender, IDictionary<string, object> parameters)
		{
			//TODO Put your code here and update the summary above
		}

		public void OnResponse(IRobotCommandResponse response)
		{
			Debug.WriteLine(response.ResponseType + ": " + response.Status);
		}
		private void LEDResponse(IRobotCommandResponse response)
		{
			Debug.WriteLine("led response: " + response.Status);
		}
		private void PlayAudioResponse(IRobotCommandResponse response)
		{
			Debug.WriteLine("Play audio response: " + response.Status);
		}

		private void DriveArcResponse(IRobotCommandResponse response)
		{
			Debug.WriteLine("drive arc response: " + response.Status);
		}
		private void DriveResponse(IRobotCommandResponse response)
		{
			Debug.WriteLine("drive response: " + response.Status);
		}


		
	

		#region IDisposable Support
		private bool _isDisposed = false;

		private void Dispose(bool disposing)
		{
			if (!_isDisposed)
			{
				if (disposing)
				{
					// TODO: dispose managed state (managed objects).
				}

				// TODO: free unmanaged resources (unmanaged objects) and override a finalizer below.
				// TODO: set large fields to null.

				_isDisposed = true;
			}
		}

		// TODO: override a finalizer only if Dispose(bool disposing) above has code to free unmanaged resources.
		// ~MistySkill() {
		//   // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
		//   Dispose(false);
		// }

		// This code added to correctly implement the disposable pattern.
		public void Dispose()
		{
			// Do not change this code. Put cleanup code in Dispose(bool disposing) above.
			Dispose(true);
			// TODO: uncomment the following line if the finalizer is overridden above.
			// GC.SuppressFinalize(this);
		}
		#endregion
	}
}
