using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using MistyRobotics.SDK.Events;
using MistyRobotics.SDK.Messengers;

namespace MistyCSharpSkill2
{
	class MovementHistory
	{

		// input buffer variable to store 10 most recent inputs
		List<MoveCommand> inputBuffer;
		Stopwatch stopwatch;
		DateTimeOffset previousTime, currentTime;

		public int size { get; private set; }

		// constructor (kinda pointless)
		public MovementHistory(IDriveEncoderEvent firstEvent)
		{
			inputBuffer = new List<MoveCommand>();
			stopwatch = new Stopwatch();
			stopwatch.Start();
			size = 500;
			previousTime = firstEvent.Created;
		}


		// add an input to the end of the list
		public void Enqueue(IDriveEncoderEvent driveEncoderData)
		{
			//IDriveEncoderEvent inputTime = new IDriveEncoderEvent(input, stopwatch.ElapsedMilliseconds);
			previousTime = currentTime;
			currentTime = driveEncoderData.Created;
			TimeSpan difference = currentTime.Subtract(previousTime);
		
			MoveCommand moveCommand = new MoveCommand(driveEncoderData, difference.TotalMilliseconds);
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

		public void RetraceSteps(IRobotMessenger _misty)
        {
			int size = inputBuffer.Count;
			
			for (int i = 0; i < size; i++)
            {
				MoveCommand driveEvent = Pop();
				_misty.DriveTrack(driveEvent.driveEncoderData.LeftVelocity, driveEvent.driveEncoderData.RightVelocity, null);
				Thread.Sleep((int)driveEvent.millisecondsToDriveFor);

			}
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

