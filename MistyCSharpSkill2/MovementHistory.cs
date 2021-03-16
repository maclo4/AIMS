using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MistyRobotics.Common.Data;
using MistyRobotics.SDK.Events;
using MistyRobotics.SDK.Messengers;
using MistyRobotics.SDK.Responses;

namespace MistyCSharpSkill2
{
	class MovementHistory
	{

		// input buffer variable to store 10 most recent inputs
		List<IRobotCommandEvent> moveQueue;
		Stopwatch stopwatch;
		DateTimeOffset previousTime, currentTime;
		bool retracingSteps = false;

		public int size { get; private set; }

		// constructor (kinda pointless)
		public MovementHistory(DateTimeOffset firstEvent)
		{
			moveQueue = new List<IRobotCommandEvent>();
			stopwatch = new Stopwatch();
			stopwatch.Start();
			size = 10000;
			previousTime = firstEvent;
			currentTime = previousTime;
		}


		// add an input to the end of the list
		public void Enqueue(IRobotCommandEvent moveCommand)
		{
			if (retracingSteps == true)
			{
				//Debug.WriteLine("Command rejected bc of retracing steps being active");
				return;
			}

			//IDriveEncoderEvent inputTime = new IDriveEncoderEvent(input, stopwatch.ElapsedMilliseconds);
			//previousTime = currentTime;
			//currentTime = _created;
			//TimeSpan difference = currentTime.Subtract(previousTime);

			/*
			Debug.WriteLine("driveEncoderData.Created: " + driveEncoderData.Created);
			Debug.WriteLine("Previous Time: " + previousTime);
			Debug.WriteLine("Current Time: " + currentTime);
			Debug.WriteLine("difference: " + difference.TotalMilliseconds);
			*/


			//MoveCommand moveCommand = new MoveCommand(_angularVelocity, _linearVelocity, _created);
			moveQueue.Add(moveCommand);

			while (moveQueue.Count > size)
			{

				moveQueue.RemoveAt(0);
			}
		}
		public IRobotCommandEvent Pop()
		{
			var movement = moveQueue.ElementAt(moveQueue.Count - 1);
			moveQueue.RemoveAt(moveQueue.Count - 1);
			return movement;
		} 

		public void RetraceSteps(IRobotMessenger _misty, int stepsToRetrace = -1)
        {
			HazardSettings hazardSettings = new HazardSettings();
			hazardSettings.DisableTimeOfFlights = true;
			hazardSettings.DisableBumpSensors = true;
			_misty.UpdateHazardSettings(hazardSettings, null);

			currentTime = DateTimeOffset.Now;

			retracingSteps = true;
			if(stepsToRetrace >= moveQueue.Count || stepsToRetrace == -1)
            {
				stepsToRetrace = moveQueue.Count;
            }
			//int size = inputBuffer.Count;
			Debug.WriteLine("size: " + size);
			for (int i = 0; i < stepsToRetrace; i++)
            {

                IRobotCommandEvent moveCommand = Pop();

				TimeSpan millisecondsToDriveFor = currentTime.Subtract(moveCommand.Created);
				currentTime = moveCommand.Created;

				if (moveCommand.Command == "Drive" || moveCommand.Command == "DriveAsync") {
					
					
					
					var linearVelocityString = moveCommand.Parameters["LinearVelocity"];
					var angularVelocityString = moveCommand.Parameters["AngularVelocity"];
					double linearVelocity = Convert.ToDouble(linearVelocityString);
					double angularVelocity = Convert.ToDouble(angularVelocityString);

					Debug.WriteLine("MoveCommand[" + i + "] Drive() Linear Velocity: " + (double)linearVelocity * -1 + ", Angular Velocity: " + (double)angularVelocity * -1 + " , ms: " + (int)millisecondsToDriveFor.TotalMilliseconds);
					_misty.DriveTime(linearVelocity * -1, angularVelocity * -1, (int)millisecondsToDriveFor.TotalMilliseconds, DriveTrackResponse);

					Thread.Sleep((int)millisecondsToDriveFor.TotalMilliseconds + 500);
				}
				else if(moveCommand.Command == "DriveTime" || moveCommand.Command == "DriveTimeAsync") 
				{
					var linearVelocity = Convert.ToDouble(moveCommand.Parameters["LinearVelocity"]);
					var angularVelocity = Convert.ToDouble(moveCommand.Parameters["AngularVelocity"]);
					var timeMs = (int)Convert.ToInt64(moveCommand.Parameters["TimeMs"]);
					Debug.WriteLine("MoveCommand[" + i + "] DrivTime() Linear Velocity: " + (double)linearVelocity * -1 + ", Angular Velocity: " + (double)angularVelocity * -1 + " , ms: " + (int)millisecondsToDriveFor.TotalMilliseconds);
					_misty.DriveTime(linearVelocity * -1, angularVelocity * -1, (int)millisecondsToDriveFor.TotalMilliseconds, DriveTrackResponse);
					Thread.Sleep((int)millisecondsToDriveFor.TotalMilliseconds + 500);
				}
				else if(moveCommand.Command == "Stop" || moveCommand.Command == "StopAsync")
                {
					Debug.WriteLine("MoveCommand[" + i + "] Stop() Linear Velocity: 0, Angular Velocity: 0 , ms: " + (int)millisecondsToDriveFor.TotalMilliseconds);
					_misty.DriveTime(0, 0, (int)millisecondsToDriveFor.TotalMilliseconds, DriveTrackResponse);
					Thread.Sleep((int)millisecondsToDriveFor.TotalMilliseconds + 500);
				}
				else if(moveCommand.Command == "DriveArc" || moveCommand.Command == "DriveArcAsync")
                {
					var heading = Convert.ToDouble(moveCommand.Parameters["Heading"]);
					var radius = Convert.ToDouble(moveCommand.Parameters["Radius"]);
					var timeMs = (int)Convert.ToInt64(moveCommand.Parameters["TimeMs"]);
                    var reverse = Convert.ToBoolean(moveCommand.Parameters["Reverse"]);
					Debug.WriteLine("MoveCommand[" + i + "] DriveArc() Heading: " + heading + ", Radius: " + radius + ", TimeMs: " + timeMs + ", Reverse: " + reverse);
					_misty.DriveArc(heading, radius, timeMs, !reverse, DriveTrackResponse);

					Thread.Sleep((int)millisecondsToDriveFor.TotalMilliseconds + 500);
				}

            }

			

			hazardSettings.DisableTimeOfFlights = false;
			hazardSettings.RevertToDefault = true;
			_misty.UpdateHazardSettings(hazardSettings, null);

			retracingSteps = false;
		}

        private void DriveTrackResponse(IRobotCommandResponse commandResponse)
        {
			Debug.WriteLine("Drive track resonse: " + commandResponse.Status);
        }

        /*
		// returns a specified amount of the most recent inputs from the list
		public List<InputTime> getTopInputs(int numInputs = 3)
		{
			List<InputTime> topInputs = new List<InputTime>(numInputs);
			int i = inputBuffer.Count - 1;

			while (i >= inputBuffer.Count - numInputs && i >= 0)
			{
				topInputs.Add(inputBuffer[i]);
				i--;
			}

			return topInputs;
		}
		*/

        // mainly for testing
        /*
		public void printBuffer()
		{
			List<InputTime> printTop = getTopInputs(20);
			int i = 0;
			foreach (InputTime inputs in printTop)
			{
				UnityEngine.Debug.Log(i + ": " + inputs.input + ", " + inputs.elapsedMilliseconds);
				i++;
			}
		}
		*/

        public long getCurrentTime()
		{
			return stopwatch.ElapsedMilliseconds;
		}

	}
}

