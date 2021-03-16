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
    class MoveCommands
    {
        static AutoResetEvent autoEvent;//= new AutoResetEvent(false);
        IRobotMessenger _misty;
        public bool isMovingFromHazard;
        
        public MoveCommands(IRobotMessenger misty, AutoResetEvent AutoEvent)
        {
            _misty = misty;
            autoEvent = AutoEvent;
        }
        public void Drive(double linearVelocity, double angularVelocity, ProcessCommandResponse commandCallback, bool wasCalledFromHazard = false)
        {
            
            if(isMovingFromHazard == true && wasCalledFromHazard == false)
            {
                Debug.WriteLine("Ope gonna wait on the hazard to get handled");
                autoEvent.WaitOne();

                Debug.WriteLine("signal received!!1!");
            }
            _misty.Drive(linearVelocity, angularVelocity, commandCallback);
            Debug.WriteLine("drive command ordered");
        }
        public void DriveTime(double linearVelocity, double angularVelocity, int timeMs, ProcessCommandResponse commandCallback, bool wasCalledFromHazard = false)
        {
            if (isMovingFromHazard == true && wasCalledFromHazard == false)
            {
                autoEvent.WaitOne();
            }
            _misty.DriveTime(linearVelocity, angularVelocity, timeMs, commandCallback);
        }
        public void DriveArc(double heading, double radius, int timeMs, bool reverse, ProcessCommandResponse commandCallback, bool wasCalledFromHazard = false)
        {
            if (isMovingFromHazard == true && wasCalledFromHazard == false)
            {
                autoEvent.WaitOne();
            }
            _misty.DriveArc(heading, radius, timeMs, reverse, commandCallback);
        }
    }
}
