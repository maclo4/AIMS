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
	/// <summary>
	/// Class that keeps track of past movement commands and the time that each was created. 
	/// Allows for retracing of steps.
	/// </summary>
	class MovementHistory
	{
		/// <summary>
		///  input buffer variable to store 10 most recent inputs
		/// </summary>
		List<IRobotCommandEvent> moveQueue;
		DateTimeOffset previousTime, currentTime;
		bool retracingSteps = false;

		public int size { get; private set; }

		/// <summary>
		/// Initialize variables with a the time of the first command created
		/// </summary>
		/// <param name="firstEvent">The Created field of a misty command</param>
		public MovementHistory(DateTimeOffset firstEvent)
		{
			moveQueue = new List<IRobotCommandEvent>();
			size = 10000;
			previousTime = firstEvent;
			currentTime = previousTime;
		}


		
		/// <summary>
		/// Add a movement command to the queue.
		/// </summary>
		/// <param name="moveCommand">Movement command to be added to the queue</param>
		public void Enqueue(IRobotCommandEvent moveCommand)
		{
			if (retracingSteps == true)
			{
				return;
			}

			moveQueue.Add(moveCommand);

			// remove oldest command if queue gets too big. Unlikely
			while (moveQueue.Count > size)
			{
				moveQueue.RemoveAt(0);
			}
		}

		/// <summary>
		/// Pop the top event off the queue
		/// </summary>
		/// <returns></returns>
		public IRobotCommandEvent Pop()
		{
			var movement = moveQueue.ElementAt(moveQueue.Count - 1);
			moveQueue.RemoveAt(moveQueue.Count - 1);
			return movement;
		} 

		/// <summary>
		/// Redo the commands in the queue in reverse to retrace steps
		/// </summary>
		/// <param name="_misty">The IRobotMessenger representing the current Misty in use</param>
		/// <param name="stepsToRetrace">Number of commands in the queue to retrace</param>
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

   
	}
}

