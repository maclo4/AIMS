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
using MistyCSharpSkill2;
using System.Threading;
using System.ComponentModel;
//using System.Windows.Forms;


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
		private MovementHistory movementHistory;

		private bool firstMove = true;

		private readonly object balanceLock = new object();
		/// <summary>
		/// This is used as a signal to prevent misty from receiving movement commands when she is already moving from a hazard.
		/// Set to true at the start of a hazard processing and false at the end of a hazard processing.
		/// </summary>

		// TODO: make this thread safe
		public static bool isMovingFromHazard{ get; set;}

		/// <summary>
		/// This is used as a signal to prevent misty from receiving movement commands when she is already moving from a hazard.
		/// Set to true at the start of a hazard processing and false at the end of a hazard processing.
		/// </summary>
		private bool isMovingFromFrontHazard = false;

		/// <summary>
		/// This is used as a signal to prevent misty from receiving movement commands when she is already moving from a hazard.
		/// Set to true at the start of a hazard processing and false at the end of a hazard processing.
		/// </summary>
		private bool isMovingFromBackHazard = false;

		/// <summary>
		/// This is used as a signal to prevent misty from receiving movement commands when she is already moving from a hazard.
		/// Set to true at the start of a hazard processing and false at the end of a hazard processing.
		/// </summary>
		private bool isMovingFromFrontAndBackHazard = false;


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


		private enum HazardStates { Front, Back, FrontAndBack, NA};
		
		/// <summary>
		/// This indicates what direction the current hazard is coming from
		/// </summary>
		HazardStates hazardStates;

		IGetSlamStatusResponse slamStatus;

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
		private static IsSpinning isSpinning = IsSpinning.NotSpinning;

		/// <summary>
		/// This is used in spinTillOpenArea() and closestTOFSensorReading() to indicate spin direction
		/// </summary>
		TimeOfFlightPosition initialClosestSensor = TimeOfFlightPosition.Unknown;


		/// <summary>
		/// This is set by the "closestTOFReading()" function. It represents the closest object to any of the 3 front tof sensors.
		/// </summary>
		private double closestObject = -1;

		/// <summary>
		/// Current distance reading of the frontRight time of flight sensor
		/// </summary>
		private double frontRightTOF = -1;

		/// <summary>
		/// Current distance reading of the frontLeft time of flight sensor
		/// </summary>
		private double frontLeftTOF = -1;

		/// <summary>
		/// Current distance reading of the frontCenter time of flight sensor
		/// </summary>
		private double frontCenterTOF = -1;

		/// <summary>
		/// Current distance reading of the back time of flight sensor
		/// </summary>
		private double backTOF = -1;

		/// <summary>
		/// Amount of move commands given in this sequence of roaming
		/// </summary>
		int stepsToRetrace = 0;

		/// <summary>
		/// Timer that ensures dumb roaming stops after a certain amount of time 
		/// </summary>
		System.Timers.Timer dumbRoamTimer = new System.Timers.Timer(1000);

		/// <summary>
		/// This is used to ensure that no commands are issued while hazards are happening
		/// </summary>
		AutoResetEvent autoResetEvent = new AutoResetEvent(false);

		/// <summary>
		/// This class is a wrapper class for misty move commands. It takes in the same parameters but prevents movement while hazards are being handled
		/// </summary>
		MoveCommands moveCommands;

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
		public void OnStart(object sender, IDictionary<string, object> parameters)
		{
			System.Threading.Thread.CurrentThread.Name = "MainThread";
			
			dumbRoamTimer.Elapsed += new ElapsedEventHandler(OnTimedEvent);
			dumbRoamTimer.AutoReset = true;
			dumbRoamTimer.Start();

		
			moveCommands = new MoveCommands(_misty, autoResetEvent);

			// Check description (All functions should have descriptions if you hover over them)
			registerMistyEvents();

			
			Debug.WriteLine("Current thread: " + System.Threading.Thread.CurrentThread.ManagedThreadId);

			// =============================================================================================================
			// Temporary code, isolating certain parts of the code to test more efficiently
			// =============================================================================================================


			// Debug.WriteLine("Retracing complete");
			//while (true) { }

			// =============================================================================================================
			// \/ Real code begins again here \/ 
			// =============================================================================================================

			// sometimes they get changed, just make sure theyre all set back to normal
			SetDefaultHazardSettings();
			
			// just make sure the variables for the sensors all have a reading
			initializeSensorReadings();

			for (int i = 0; i < 5; i++)
			{
				dumbRoaming();
				moveCommands.DriveArc(IMUData.Yaw - 90, 0, 3000, false, null);
				System.Threading.Thread.Sleep(3500);
			}

			while (true) { }

			do
			{
				Debug.WriteLine("Starting mapping (again)");
				initializeMapping();
			} while (!(isExploring && hasPose && isStreaming));

			// again, ensure that misty has a pose
			if (isExploring && hasPose && isStreaming)
			{
				
				//IGetMapResponse mapResponse;

				// keep driving in circles till the map has some sort of content
				do
				{
					Debug.WriteLine("Trying to get initial map (size > 0)");
					drive360();
					//mapResponse = await _misty.GetMapAsync();
					_misty.GetMap(GetMapResponse);
					//map = mapResponse.Data;
					Debug.WriteLine("Map size: " + map.Size);
					Debug.WriteLine("Map width: " + map.Width);

				} while (map.Size == 0);

				initialPose = pose;
				Debug.WriteLine("initial pose: " + initialPose);

				// not sure if I should be turning on tracking or not
				for (int i = 0; i < 5; i++)
				{
					dumbRoaming();
					moveCommands.DriveArc(IMUData.Yaw - 90, 0, 3000, false, null);
					System.Threading.Thread.Sleep(3500);
				}


				/*
				await _misty.StartTrackingAsync();
				await semiSmartRoam();
				*/

				// I think this is unneccesary now
				int width = map.Width;
				int height = map.Height;

				var bitMap = setByteArray();
				setBitmap(bitMap);


				// just make sure to stop mapping and streaming when the program ends or else misty will just keeping going
				_misty.StopMapping(null);
				_misty.StopSlamStreaming(null);

			}


			// change the leds to show that the program is over
			_misty.ChangeLED(0, 0, 200, LEDResponse);

			// Cancel the skill
			_misty.CancelRunningSkill("93e9b284-b07d-443d-8339-64f96b483b07", null);

		}

		/// <summary>
		/// Just make sure that all the sensor variables are initialized
		/// </summary>
		private void initializeSensorReadings()
        {
			Debug.WriteLine("Getting initial sensor readings...");
			while (IMUEventReceived == false || frontCenterTOF == -1 || frontLeftTOF == -1 || frontRightTOF == -1 || backTOF == -1) { }
       
        }

		/// <summary>
		/// callback function for the hazard event. Not really necessary but doesnt do hard
		/// </summary>
		/// <param name="commandResponse">Standard IRobotCommandResponse</param>
        private void OnHazardEvent(IRobotCommandResponse commandResponse)
        {
			if(System.Threading.Thread.CurrentThread.Name == null)
            {
				Thread.CurrentThread.Name = "HazardEventThread";
            }
	
		}

		/// <summary>
		/// Any time that misty receives any command it will go to here. We are only really interested in the movement commands. 
		/// </summary>
		/// <param name="sender"></param>
		/// <param name="commandEvent"></param>
        private void ProcessRobotCommandEvent(object sender, IRobotCommandEvent commandEvent)
        {

			// ensure that its a drive command
			if( commandEvent.Command == "Drive" || commandEvent.Command == "DriveTime" || 
				commandEvent.Command == "DriveAsync" || commandEvent.Command == "DriveTimeAsync" ||
				commandEvent.Command == "DriveHeading" || commandEvent.Command == "DriveHeadingAsync" ||
				commandEvent.Command == "DriveTrack" || commandEvent.Command == "DriveTrackAsync" || commandEvent.Command == "Stop")
            {
				lock (balanceLock)
				{
					//increment steps to retrace bc youre also about to add this movement to the MovementHistory queue
					stepsToRetrace++;
					Debug.WriteLine("Robot Command Event: " + commandEvent.Command);

					// the first command is used to initialize the movement history. TEST: DONE BUT still test a bit more
					if (movementHistory == null) //firstMove == true
					{
						Debug.WriteLine("Movement history initialized");
						movementHistory = new MovementHistory(commandEvent.Created);
						//firstMove = false;
					}

					movementHistory.Enqueue(commandEvent);
				}
			}
			// this else if is the same as above except it handles drive arc commands
			else if(commandEvent.Command == "DriveArc" || commandEvent.Command == "DriveArcAsync")
            {
				Debug.WriteLine("Robot Command Event: " + commandEvent.Command);
				stepsToRetrace++;
				if (movementHistory == null)
				{
					movementHistory = new MovementHistory(commandEvent.Created);
					firstMove = false;
				}
				// for retracing these, the robot should trace back to the original heading
				IRobotCommandEvent driveArcCommand = commandEvent;
				driveArcCommand.Parameters["Heading"] = IMUData.Yaw; 
				movementHistory.Enqueue(driveArcCommand);
			}
			

		}


		/// <summary>
		/// Simply update the pose variable to make it current.
		/// </summary>
		private void ProcessSelfStateEvent(object sender, ISelfStateEvent selfStateEvent)
		{
			pose = selfStateEvent.Pose;
		}

		/// <summary>
		/// Set isExploring, hasPose, isStreaming, and isTracking to reflect misty's current slam status.
		/// If misty does not have a pose yet, find it.
		/// Consider changing this from booleans to some other data type that could combine 4 variables into 1.
		/// </summary>
		private void ProcessSlamStatusEvent(object sender, ISlamStatusEvent slamStatusEvent) 
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

			// TODO: The mapping just stopped working so Im not sure if I'll need this. It seems like I shouldnt need it with properly working hardware bc 
			// misty already retraces her steps. So if she loses pose, eventually shell find it again onces shes in the middle of the room
			if(lostPose == true && currentlyFindingPose == false)
            {
				// still very much working on this part so it is commented out for now
				//await findPose();
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
		/// 
		/// </summary>
		/// <param name="sender"></param>
		/// <param name="hazardEvent"></param>
		private void ProcessHazardEvent(object sender, IHazardNotificationEvent hazardEvent)
		{
			// think of hazardStates as a static variable. Every time a new hazard is sent it will get updated, and the thread that is 
			// handling the hazard will accordingly react. This prevents there being like 10 different hazard processings going on at once
			if (hazardEvent.FrontHazard && hazardEvent.BackHazard)
            {
				hazardStates = HazardStates.FrontAndBack;
				Debug.WriteLine("hazardStates = front and back");

			}
			else if(hazardEvent.FrontHazard)
            {
				hazardStates = HazardStates.Front;
				Debug.WriteLine("hazardStates = front");
			}
			else if(hazardEvent.BackHazard)
            {
				hazardStates = HazardStates.Back;
				Debug.WriteLine("hazardStates = back");
			}

		
			// Don't handle one hazard while another is already being worked on.
			if (!isMovingFromHazard )
			{
				//TODO: turn these 2 variables to 1. 
				isMovingFromHazard = true;
				moveCommands.isMovingFromHazard = true;

				// this is actually pretty nicely broken into small pieces. Check the current hazard and call the correct function
				if (hazardStates == HazardStates.FrontAndBack)
				{
					// front and back handling
					Debug.WriteLine("Handling front and back hazard");
					handleFrontAndBackHazard();
				
				}
				else if (hazardStates == HazardStates.Front)
				{
					// front handling
					Debug.WriteLine("Handling front hazard");
					handleFrontHazard();
				
				}
				else if (hazardStates == HazardStates.Back)
				{
					// back handling
					Debug.WriteLine("Handling back hazard");
					handleBackHazard();
				
				}

				hazardStates = HazardStates.NA;
				isMovingFromHazard = false;
				moveCommands.isMovingFromHazard = false;

				// IMPORTANT: drive commands will be waiting to execute until the autoresetevent does set()
				autoResetEvent.Set();
			}

			
		}


		/// <summary>
		/// If there is a hazard in front and behind, theres usually not much you can do. This function turns off tof hazards 
		/// to allow misty to move and then tries to find its way out.
		/// </summary>
		private void handleFrontAndBackHazard()
        {
			bool hazardHandledSuccessfully = false;
			bool openAreaFound;
			do {

				Debug.WriteLine("Handling front and back hazard");

				// if there is a front and back hazard, misty will not be able to spin unless tof snesors are disabled.
				disableTOFHazards();
				hazardHandledSuccessfully = moveAwayFromObstable(HazardStates.FrontAndBack);
				
				Debug.WriteLine("hazard spinnin");
				openAreaFound = spinTillOpenArea(1.25, true);

			} while(openAreaFound == false);

			_misty.DriveTime(10, 0, 3000, null);
			SetDefaultHazardSettings();
			System.Threading.Thread.Sleep(3100);
		}

		/// <summary>
		/// Attempts to move away from front hazard 
		/// </summary>
		private void handleFrontHazard()
        {
			bool hazardHandledSuccessfully = false;
			Debug.WriteLine("Attempting to drive backwards");
			hazardHandledSuccessfully = moveAwayFromObstable(HazardStates.Front);

			// check if driving backwards caused any more hazards
			if(hazardStates == HazardStates.Back || hazardStates == HazardStates.FrontAndBack || hazardHandledSuccessfully == false)
            {
				// handle back hazard
				 handleFrontAndBackHazard();
			}
            else
            {
				disableTOFHazards();
				Debug.WriteLine("hazard spinnin");
				spinTillOpenArea(1.25, true);
				SetDefaultHazardSettings();
            }

		}

		/// <summary>
		/// Attempts to move away from back hazard using moveAwayFromObstacle() then turns away from it using spinTillOpenArea()
		/// </summary>
		private void handleBackHazard()
        {
			bool hazardHandledSuccessfully = false;
			Debug.WriteLine("Attempting to drive forward");
			//_misty.DriveTime(10, 0, 3000, null);
			//System.Threading.Thread.Sleep(3100);
			
			hazardHandledSuccessfully = moveAwayFromObstable(HazardStates.Back);
		
		

			// check if driving backwards caused any more hazards
			if (hazardStates == HazardStates.Front || hazardStates == HazardStates.FrontAndBack || hazardHandledSuccessfully == false)
			{
				handleFrontAndBackHazard();
			}
			else
			{
				disableTOFHazards();
				Debug.WriteLine("hazard spinnin");
				spinTillOpenArea(1.25, true);
				SetDefaultHazardSettings();
			}
		}

		/// <summary>
		/// Attempts to move away from front hazard 
		/// TODO: split into smaller functions. Potentially just use the logic for front and back only
		/// </summary>
		private bool moveAwayFromObstable(HazardStates hazardState)
        {
			Stopwatch stopwatch = new Stopwatch();
			int timeOutMs = 7000;
			if (hazardState == HazardStates.Front)
			{
				closestTOFSensorReading();
				_misty.Drive(-5, 0, null);

				Debug.WriteLine("pre FRONT while loop backtof and closest object: " + backTOF + ", " + closestObject);
				stopwatch = Stopwatch.StartNew();
				while ((closestObject < .215 && backTOF > .07 ) && stopwatch.ElapsedMilliseconds < timeOutMs)
				{
					
					closestTOFSensorReading();
					//Debug.WriteLine("Stuck in the while loop driving backward: " + backTOF + ", " + closestObject);
				}
				stopwatch.Stop();
				_misty.Drive(0, 0, null);

				if(stopwatch.ElapsedMilliseconds > timeOutMs && closestObject <= .215)
                {
					return false;
                }
			}
			else if(hazardState == HazardStates.Back)
            {
				closestTOFSensorReading();
				_misty.Drive(5, 0, null);
				Debug.WriteLine("pre BACK while loop backtof and closest object: " + backTOF + ", " + closestObject);
				stopwatch = Stopwatch.StartNew();
				while (closestObject > .07 && backTOF < .215 && stopwatch.ElapsedMilliseconds < timeOutMs)
				{
					closestTOFSensorReading();
				}
				_misty.Drive(0, 0, null);

				if (stopwatch.ElapsedMilliseconds > timeOutMs && backTOF <= .215)
				{
					return false;
				}
			}
			else if(hazardState == HazardStates.FrontAndBack)
            {
				closestTOFSensorReading();
			
				
				Debug.WriteLine("trying to find middle point between two hazards");
				if(closestObject > backTOF)
                {
					Debug.WriteLine("back tof is closer: " + backTOF + ", " + closestObject);
					_misty.Drive(2, 0, null);
					Debug.WriteLine("pre FRONT AND BACK while loop backtof and closest object: " + backTOF + ", " + closestObject);
					stopwatch = Stopwatch.StartNew();
					while (backTOF <= closestObject && backTOF < .215 || stopwatch.ElapsedMilliseconds > timeOutMs) // !(backTOF <= closestObject + .05 && backTOF >= closestObject - .05 )
					{
						closestTOFSensorReading();
						//Debug.WriteLine("Stuck in the while loop driving forward: " + backTOF + ", " + closestObject);
					}
					Debug.WriteLine("After driving: " + backTOF + ", " + closestObject);
					_misty.Drive(0, 0, null);

					if (stopwatch.ElapsedMilliseconds > timeOutMs)
					{
						return false;
					}
				}
                else
                {
					Debug.WriteLine("front tof is closer: " + backTOF + ", " + closestObject);
					_misty.Drive(-2, 0, null);
					Debug.WriteLine("pre FRONT AND BACK while loop backtof and closest object: " + backTOF + ", " + closestObject);
					stopwatch = Stopwatch.StartNew();
					while (closestObject <= backTOF && closestObject < .215 || stopwatch.ElapsedMilliseconds > timeOutMs) // !(closestObject <= backTOF + .05 && closestObject >= backTOF - .05)
					{
						closestTOFSensorReading();
						// Debug.WriteLine("Stuck in the while loop driving backward: " + backTOF + ", " + closestObject);
					}
					Debug.WriteLine("After driving: " + backTOF + ", " + closestObject);
					_misty.Drive(0, 0, null);

					if (stopwatch.ElapsedMilliseconds > timeOutMs)
					{
						return false;
					}
				}
            }
			return true;
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
			else if(TimeOfFlightEvent.SensorPosition == TimeOfFlightPosition.Back)
            {
				
				backTOF = TimeOfFlightEvent.DistanceInMeters;
				//Debug.WriteLine("back tof updooted: " + backTOF);
			}
		}


		/// <summary>
		/// turn misty's hazard settings back to the default
		/// </summary>
		private void SetDefaultHazardSettings()
		{
			HazardSettings hazardSettings = new HazardSettings();
			hazardSettings.DisableTimeOfFlights = false;
			hazardSettings.RevertToDefault = true;
			_misty.UpdateHazardSettings(hazardSettings, null);
		}

		/// <summary>
		/// This makes it so misty won't trigger hazards from timeofflight sensors
		/// </summary>
		private void disableTOFHazards()
		{
			HazardSettings hazardSettings = new HazardSettings();
			hazardSettings.DisableTimeOfFlights = true;
			_misty.UpdateHazardSettings(hazardSettings, null);
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
		private bool spinTillOpenArea(double distance, bool wasCalledFromHazard = false)
		{
			// TODO: remove all threading related debugging. Might be useful for a bit longer though
			System.Threading.Thread CurrentThread = System.Threading.Thread.CurrentThread;
			Debug.WriteLine("Starting Spinntillopenarea(), hazard state = " + wasCalledFromHazard + ", currentThread = " + CurrentThread.Name);

			Stopwatch stopwatch = Stopwatch.StartNew();
			int msElapsed;


			// flag to let other threads n stuff know that spinning has begun
			// TODO: whats a better way to have flags besides booleans?? It seems like microsoft is suggesting basically this. Look at IsBusy
			// https://docs.microsoft.com/en-us/dotnet/standard/asynchronous-programming-patterns/best-practices-for-implementing-the-event-based-asynchronous-pattern
			isSpinning = IsSpinning.Spinning;

			// get current closest front object
			closestTOFSensorReading();
			
			int i = 0;

			// keep looping till closest object is further than the set distance
			while (i < 5 && closestObject < distance)
			{
				msElapsed = 0;

				//TODO: test the autoresetevent
				if(wasCalledFromHazard == false && isMovingFromHazard == true ) 
				{
					autoResetEvent.WaitOne();
				}

				// basically this code is checking the initial direction to turn then its gonna keep turning in that direction till it finds open area
				if (i == 0 || initialClosestSensor == TimeOfFlightPosition.Unknown)
				{
					Debug.WriteLine("getting initial turn direction");
					initialClosestSensor = turnDirection();

					if (initialClosestSensor == TimeOfFlightPosition.FrontLeft)
					{	
						moveCommands.DriveArc(IMUData.Yaw - 90, 0, 7500, false, null, wasCalledFromHazard);
					}
					else if (initialClosestSensor == TimeOfFlightPosition.FrontRight)
					{
						moveCommands.DriveArc(IMUData.Yaw + 90, 0, 7500, false, null, wasCalledFromHazard);
					}
				}
				else if (initialClosestSensor == TimeOfFlightPosition.FrontLeft)
				{
					moveCommands.DriveArc(IMUData.Yaw - 90, 0, 7500, false, null, wasCalledFromHazard);
				}
				else if (initialClosestSensor == TimeOfFlightPosition.FrontRight)
				{
					moveCommands.DriveArc(IMUData.Yaw + 90, 0, 7500, false, null, wasCalledFromHazard);
				}
				stopwatch = Stopwatch.StartNew();

				// output the current sensor readings and other important information
				Debug.WriteLine("Closest Object: " + closestObject);
				Debug.WriteLine("frontlefttof: " + frontLeftTOF);
				Debug.WriteLine("frontRighttof: " + frontRightTOF);
				Debug.WriteLine("frontCentertof: " + frontCenterTOF);
				Debug.WriteLine("i = " + i);
				Debug.WriteLine("hazard state = " + wasCalledFromHazard);
				Debug.WriteLine("");

				do
				{
					closestTOFSensorReading();
				
					//msElapsed = msElapsed + 100;
					System.Threading.Thread.Sleep(100);
				} while (closestObject < distance && stopwatch.ElapsedMilliseconds <= 7500); //&& degreesTurned < 355  && driveArcResponse.Status == ResponseStatus.Success
				i++;

			}

			stopwatch.Stop();

			// stop spinning after object is found or loop times out
			moveCommands.Drive(0, 0, null, wasCalledFromHazard);
            
			// if no open area found, move away from obstacle 
			// TODO: implement moveawayfromobstacle here
			if (closestObject < distance) //msElapsed >= 29000 ||  // degreesTurned >= 355
			{
				
				
				if (closestObject > backTOF)
				{
					Debug.WriteLine("Moving forward bc the last spin didnt reveal any open areas");
					moveAwayFromObstable(HazardStates.Back);
					//moveCommands.DriveTime(10, 0, 2500, DriveArcResponse, wasCalledFromHazard);
					//System.Threading.Thread.Sleep(2500);

				}
				else
				{
					Debug.WriteLine("Moving backward bc the last spin didnt reveal any open areas");
					moveAwayFromObstable(HazardStates.Front);
					//moveCommands.DriveTime(-10, 0, 2500, DriveArcResponse, wasCalledFromHazard);
					//System.Threading.Thread.Sleep(2500);
				}

				return false;
			}
			
			Debug.WriteLine("Open area to drive was found!");

			// reset flags
			isSpinning = IsSpinning.NotSpinning;
			initialClosestSensor = TimeOfFlightPosition.Unknown;

			return true;

		}



		/// <summary>
		/// Turn in a specific direction based on which sensor has the object closest to it
		/// </summary>
		private TimeOfFlightPosition turnDirection()
        {
			switch (tofPosition) 
			{
				case TimeOfFlightPosition.FrontCenter:
					return TimeOfFlightPosition.FrontLeft;

				case TimeOfFlightPosition.FrontLeft:
					return TimeOfFlightPosition.FrontLeft;

				case TimeOfFlightPosition.FrontRight:
					return TimeOfFlightPosition.FrontRight;

			};
			return TimeOfFlightPosition.Unknown;
        }

		/// <summary>
		/// Register only certain TOF sensors so that we dont waste too much resources. Register all other event listeners here as well
		/// </summary>
		private void registerMistyEvents()
		{

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
			_misty.RegisterHazardNotificationEvent(300, true, "Hazard Event", OnHazardEvent);
			_misty.HazardNotificationEventReceived += ProcessHazardEvent;


			// Listen to the slam status event, set booleans such as "isExploring", "hasPose", "isStreaming" to true or false
			_misty.RegisterSlamStatusEvent(20, true, "Slam Status", null, null);
			_misty.SlamStatusEventReceived += ProcessSlamStatusEvent;

			//_misty.RegisterLocomotionCommandEvent(100, true, null, "Movement Command Event", null);
			//_misty.LocomotionCommandEventReceived += ProcessLocomotionCommandEvent;

			//_misty.RegisterHaltCommandEvent(100, true, "Halt Command Event", null);
			//         _misty.HaltCommandEventReceived += ProcessHaltCommandEventReceived;
		
			_misty.RegisterRobotCommandEvent(100, true, "Robot Command Event", null);
			_misty.RobotCommandEventReceived += ProcessRobotCommandEvent;


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

			List<TimeOfFlightValidation> tofBackValidations = new List<TimeOfFlightValidation>();
			tofBackValidations.Add(new TimeOfFlightValidation { Name = TimeOfFlightFilter.SensorName, Comparison = ComparisonOperator.Equal, ComparisonValue = TimeOfFlightPosition.Back });
			_misty.RegisterTimeOfFlightEvent(10, true, tofBackValidations, "Back", null);

			_misty.TimeOfFlightEventReceived += ProcessTimeOfFlightEvent;


		}

		/// <summary>
		/// Very new function meant to spin till misty finds a pose
		/// TODO: Probably delet this
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

				await _misty.DriveArcAsync(IMUData.Yaw - 10, 0, 2500, false);

				//System.Threading.Thread.Sleep(15000);
				await Task.Delay(5000);
			}


			currentlyFindingPose = false;
		}

		/// <summary>
		/// Keep restarting mapping till this finds a pose
		/// TODO: this used to work actually pretty well. Idk what happened with the camera but now I have no real way of testing this 
		/// because I cant even get it to work manually
		/// </summary>
		private void initializeMapping()
		{
			if (hasPose && isExploring && isStreaming) return;

			Debug.WriteLine("TURNING OFF AND ON MAPPING");
			_misty.StopMapping(null);
			_misty.StopSlamStreaming(null);

			_misty.StartMapping(null);
			_misty.StartObstacleDetection(300, null);
			moveCommands.DriveArc(IMUData.Yaw - 25, .05, 2500, false, null);
			//_misty.DriveArc(IMUData.Yaw - 25, .05, 2500, false, null);

			System.Threading.Thread.Sleep(15000);
			//await Task.Delay(15000);



			//IGetSlamStatusResponse slamStatus = await _misty.GetSlamStatusAsync();
			_misty.GetSlamStatus(SlamStatusResponse);
			System.Threading.Thread.Sleep(3000);
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
		/// Increment elapsed seconds and secondsSinceCommandCalled each time the timer ticks. Its kinda weird, but 
		/// it incrememnts a different amount depending on if misty is spinning or driving straight. This is because spinning
		/// takes longer and we really want to ensure that the robot goes a certain distance, not necessarily a certain time
		/// </summary>
		private static void OnTimedEvent(object source, ElapsedEventArgs e)
		{
			//Debug.WriteLine("OnTimedEvent thread: " + Thread.CurrentThread.ManagedThreadId + ", " + Thread.CurrentThread.Name);
			if (isSpinning == IsSpinning.NotSpinning)
			{
				elapsedSeconds = elapsedSeconds + 3;
				
				secondsSinceCommandCalled++;
			}
            else
            {
				elapsedSeconds++;
			}
		}

		/// <summary>
		/// One of the core functions. This basically roams in a direction for a certain amount of time while avoiding objects, then attempts to return to its original position.
		/// This takes advantage of many other functions such as spinTillOpenArea(), the MoveCommands class, MovementHistory class, and retraceSteps()
		/// </summary>
		private void dumbRoaming()
		{
			stepsToRetrace = 0;
			elapsedSeconds = 0;
			var random = new Random();
			
		
			Debug.WriteLine("Dumb Roaming started, elapsed seconds: " + elapsedSeconds + ", isMovingFromHazard = " + isMovingFromHazard);

			while (elapsedSeconds < 45) 
			{
				
				closestTOFSensorReading();
				
				// if object is far away, drive straight
				if (!isMovingFromHazard && closestObject > .75 && robotState != RobotState.DriveStraight)
				{
					
					double angularVelocity = random.Next(-10, 10);
					Debug.WriteLine("drivestraight (dumb roam)");
					moveCommands.Drive(10, 0, DriveResponse);
				
					robotState = RobotState.DriveStraight;
				
					secondsSinceCommandCalled = 0;

				
				}
				// if object is close, spinTillOpenArea()
				else if (!isMovingFromHazard && closestObject < .75)
				{
					Debug.WriteLine("find open direction (dumb roam)");

					moveCommands.Drive(0, 0, null);
					System.Threading.Thread.Sleep(500);
					
					spinTillOpenArea(1.25);

					// okay wait why is this a thing? What is it used for? Why would I do this instead of using IsSpinning
					// this is why refactoring is good lmao
					// TODO: get rid of this
					robotState = RobotState.Spinning;
					
					secondsSinceCommandCalled = 0;

					moveCommands.Drive(10, 0, DriveResponse);
					System.Threading.Thread.Sleep(500);

				}
				else if (isMovingFromHazard)
				{
					secondsSinceCommandCalled = 0;
				}
				
				// possibly just make this an else statement? Idrk, probably could just merge with the above state
				// TODO: fix this trash
				else if(driveEncoderData.LeftVelocity == 0 && driveEncoderData.RightVelocity == 0
						&& isMovingFromHazard == false && secondsSinceCommandCalled > 5) //robotState != RobotState.Hazard
				{
					// I think this is where the problem is, but it has happened in other areas as well. 
					robotState = RobotState.NA;
					secondsSinceCommandCalled = 0;
					Debug.WriteLine("Dumb Roam is not being blocked by hazards, but is neither in state 'drive360' or 'drivestraight'. Try driving backwards.");
					moveCommands.DriveTime(-10, 0, 2500, null);
					//_misty.DriveTime(-10, 0, 2500, null);
					System.Threading.Thread.Sleep(2500);
					//await Task.Delay(2500);
					spinTillOpenArea(1.25);

					//roamStates = DumbRoamStates.NA;
					
                }
			
				
			}
			moveCommands.Drive(0, 0, null);
			System.Threading.Thread.Sleep(2000);

			Debug.WriteLine("ROAMING IS DONE WE ARE NOW RETRACING STEPS!!!!!! = " + stepsToRetrace);
			movementHistory.RetraceSteps(_misty, stepsToRetrace );
			
			elapsedSeconds = 0;
		}




		/// <summary>
		/// This turns the map into an image that is stored in the same location as the skill.
		/// You can use scp -r administrator@<ip_of_misty>:Address that was output by "Debug.WriteLine("File path: " + outputFile.Path);" to copy all of the maps created by this program.
		/// I was soo proud of this. Turns out it was unnecessary. Still gonna use it because I worked hard on this lol
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
		private void drive360()
		{

			while (!IMUEventReceived) { }
			moveCommands.DriveArc(IMUData.Yaw - 180, .1, 7000, false, null);
			//_misty.DriveArc(IMUData.Yaw - 180, .1, 7000, false, null) ;

			_misty.PlayAudio("001-EeeeeeE.wav", 1, PlayAudioResponse);
			System.Threading.Thread.Sleep(7000);
			//await Task.Delay(7000);
			moveCommands.DriveArc(IMUData.Yaw - 180, .1, 7000, false, null);
			//_misty.DriveArc(IMUData.Yaw - 180, .1, 7000, false, null);
			_misty.PlayAudio("001-EeeeeeE.wav", 1, PlayAudioResponse);
			System.Threading.Thread.Sleep(7000);
			//await Task.Delay(7000);

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

			HazardSettings hazardSettings = new HazardSettings();
			hazardSettings.DisableTimeOfFlights = false;
			hazardSettings.RevertToDefault = true;
			_misty.UpdateHazardSettings(hazardSettings, null);

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

		/// <summary>
		/// Sets slamStatus to the current slam status
		/// </summary>
		/// <param name="_slamStatus"></param>
		public void SlamStatusResponse(IGetSlamStatusResponse _slamStatus)
		{
			slamStatus = _slamStatus;
        }

		/// <summary>
		/// sets "map" to the GetMapResponse response
		/// </summary>
		/// <param name="commandResponse"></param>
		private void GetMapResponse(IGetMapResponse commandResponse)
		{
			map = commandResponse.Data;
		}

		/// <summary>
		/// Generic callback function
		/// </summary>
		/// <param name="response"></param>
		public void OnResponse(IRobotCommandResponse response)
		{
			//Debug.WriteLine(response.ResponseType + ": " + response.Status);
		}

		/// <summary>
		/// Callback function for led changes. Kinda pointless
		/// </summary>
		/// <param name="response"></param>
		private void LEDResponse(IRobotCommandResponse response)
		{
			//Debug.WriteLine("led response: " + response.Status);
		}

		/// <summary>
		/// Callback function for play audio commands. Also pointless tbh
		/// </summary>
		/// <param name="response"></param>
		private void PlayAudioResponse(IRobotCommandResponse response)
		{
			//Debug.WriteLine("Play audio response: " + response.Status);
		}

		/// <summary>
		/// Callback function for drive arc. Also basically pointless
		/// </summary>
		/// <param name="response"></param>
		private void DriveArcResponse(IRobotCommandResponse response)
		{
			//Debug.WriteLine("drive arc response: " + response.Status);
		}

		/// <summary>
		/// Callback function for drive. Still pointless
		/// </summary>
		/// <param name="response"></param>
		private void DriveResponse(IRobotCommandResponse response)
		{
			//Debug.WriteLine("drive response: " + response.Status);
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
