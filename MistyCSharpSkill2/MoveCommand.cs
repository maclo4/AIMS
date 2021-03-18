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
    /// <summary>
    /// Wrapper class that can execute certain Misty movement commands with the addition of checking for a hazard state
    /// and waiting to execute movement if necessary.
    /// </summary>
    class MoveCommands
    {
        /// <summary>
        /// This will be set by hazard events once they have finished. Meaning that it can be used to pause a thread till the hazard is resolved
        /// </summary>
        static AutoResetEvent autoEvent;//= new AutoResetEvent(false);

        /// <summary>
        /// This is the IRobotMessenger that basically represents Misty and her whole sdk.
        /// </summary>
        IRobotMessenger _misty;

        /// <summary>
        /// Set by hazard events to signal if a hazard is being handled
        /// </summary>
        public bool isMovingFromHazard;
        
        /// <summary>
        /// Initialize class variables
        /// </summary>
        /// <param name="misty">Pass the _misty variable used in the rest of the program</param>
        /// <param name="AutoEvent">Pass in the AutoResetEvent used in the hazard events</param>
        public MoveCommands(IRobotMessenger misty, AutoResetEvent AutoEvent) // TODO:  ref bool _isMovingFromHazard test how refs work in c#
        {
            _misty = misty;
            autoEvent = AutoEvent;
            //isMovingFromHazard = _isMovingFromHazard;
        }

        /// <summary>
        /// Executes Misty Drive() command but checks for hazards first. Will wait till hazard is resolved to issue command.
        /// </summary>
        /// <param name="linearVelocity">Linear velocity value between -100 and 100 </param>
        /// <param name="angularVelocity">Linear velocity value between -100 and 100</param>
        /// <param name="commandCallback">Sets the callback function for the Drive command. Null is okay.</param>
        /// <param name="wasCalledFromHazard">By default this is false. Only use if calling from a hazard state</param>
        public void Drive(double linearVelocity, double angularVelocity, ProcessCommandResponse commandCallback, bool wasCalledFromHazard = false)
        {
            
            if(isMovingFromHazard == true && wasCalledFromHazard == false)
            {
                Debug.WriteLine("Ope gonna wait on the hazard to get handled");
                autoEvent.WaitOne();

                Debug.WriteLine("signal received!!1!");
            }
            _misty.Drive(linearVelocity, angularVelocity, commandCallback == null ? null : commandCallback);
            Debug.WriteLine("drive command ordered");
        }

        /// <summary>
        /// Executes Misty DriveTime() command but checks for hazards first. Will wait till hazard is resolved to issue command.
        /// </summary>
        /// <param name="linearVelocity">Linear velocity value between -100 and 100 </param>
        /// <param name="angularVelocity">Linear velocity value between -100 and 100</param>
        /// <param name="timeMs">The amount of time that Misty will drive for</param>
        /// <param name="commandCallback">Sets the callback function for the Drive command. Null is okay.</param>
        /// <param name="wasCalledFromHazard">By default this is false. Only use if calling from a hazard state</param>
        public void DriveTime(double linearVelocity, double angularVelocity, int timeMs, ProcessCommandResponse commandCallback, bool wasCalledFromHazard = false)
        {
            if (isMovingFromHazard == true && wasCalledFromHazard == false)
            {
                autoEvent.WaitOne();
            }

            _misty.DriveTime(linearVelocity, angularVelocity, timeMs, commandCallback == null ? null : commandCallback);
        }
        /// <summary>
        /// Executes Misty DriveArc() command but checks for hazards first. Will wait till hazard is resolved to issue command.
        /// </summary>
        /// <param name="heading">Misty's end rotation will be facing in this direction. Values are degrees from 0-360, but values outside this range will automatically be translated</param>
        /// <param name="radius">Radius of the arc that Misty will drive on</param>
        /// <param name="timeMs">The amount of time that Misty will drive for</param>
        /// <param name="reverse">Not actually sure what this does. Drives in reverse? </param>
        /// <param name="commandCallback">Sets the callback function for the Drive command. Null is okay.</param>
        /// <param name="wasCalledFromHazard">By default this is false. Only use if calling from a hazard state</param>
        public void DriveArc(double heading, double radius, int timeMs, bool reverse, ProcessCommandResponse commandCallback, bool wasCalledFromHazard = false)
        {
            if (isMovingFromHazard == true && wasCalledFromHazard == false)
            {
                autoEvent.WaitOne();
            }
            if (commandCallback != null)
            {
                _misty.DriveArc(heading, radius, timeMs, reverse, commandCallback);
            }
            else
            {
                _misty.DriveArc(heading, radius, timeMs, reverse, null);
            }
        }
    }
}
