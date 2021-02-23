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
		List<MoveCommand> inputBuffer;
		Stopwatch stopwatch;
		DateTimeOffset previousTime, currentTime;
		bool retracingSteps = false;

		public int size { get; private set; }

		// constructor (kinda pointless)
		public MovementHistory(DateTimeOffset firstEvent)
		{
			inputBuffer = new List<MoveCommand>();
			stopwatch = new Stopwatch();
			stopwatch.Start();
			size = 10000;
			previousTime = firstEvent;
			currentTime = previousTime;
		}


		// add an input to the end of the list
		public void Enqueue(double _angularVelocity, double _linearVelocity, DateTimeOffset _created)
		{
			if (retracingSteps == true) return;

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


			MoveCommand moveCommand = new MoveCommand(_angularVelocity, _linearVelocity, _created);
			inputBuffer.Add(moveCommand);

			while (inputBuffer.Count > size)
			{

				inputBuffer.RemoveAt(0);
			}
		}
		public MoveCommand Pop()
		{
			var movement = inputBuffer.ElementAt(inputBuffer.Count - 1);
			inputBuffer.RemoveAt(inputBuffer.Count - 1);
			return movement;
		} 

		public void RetraceSteps(IRobotMessenger _misty, int stepsToRetrace = -1)
        {
			HazardSettings hazardSettings = new HazardSettings();
			hazardSettings.DisableTimeOfFlights = true;
			_misty.UpdateHazardSettings(hazardSettings, null);

			currentTime = DateTimeOffset.Now;

			retracingSteps = true;
			if(stepsToRetrace >= inputBuffer.Count || stepsToRetrace == -1)
            {
				stepsToRetrace = inputBuffer.Count;
            }
			//int size = inputBuffer.Count;
			Debug.WriteLine("size: " + size);
			for (int i = 0; i < stepsToRetrace; i++)
            {

                MoveCommand moveCommand = Pop();

				TimeSpan millisecondsToDriveFor = currentTime.Subtract(moveCommand.created);
				currentTime = moveCommand.created;


				Debug.WriteLine("MoveCommand[" + i + "] Linear Velocity: " + moveCommand.linearVelocity + "angular velocity: " + moveCommand.angularVelocity + " , ms: " + millisecondsToDriveFor.TotalMilliseconds);
				
				_misty.DriveTime(moveCommand.linearVelocity * -100, moveCommand.angularVelocity * -1, (int)millisecondsToDriveFor.TotalMilliseconds, DriveTrackResponse);
				Thread.Sleep((int)millisecondsToDriveFor.TotalMilliseconds ); // + 500
				

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

