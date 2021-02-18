using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MistyRobotics.SDK.Events;

namespace MistyCSharpSkill2
{
    class MoveCommand
    {
       // public long millisecondsToDriveFor { get; }
        //public ILocomotionCommandEvent locomotionCommandEvent { get; }
        public double angularVelocity, linearVelocity;
        public DateTimeOffset created;
        public MoveCommand(double _angularVelocity, double _linearVelocity, DateTimeOffset _created )
        {
            //  Debug.WriteLine("ms (long): " + (long)_millisecondsToDriveFor);
            // Debug.WriteLine("ms: " + _millisecondsToDriveFor);
            // locomotionCommandEvent = _locomotionCommandEvent;
            angularVelocity = _angularVelocity;
            linearVelocity = _linearVelocity;
            created = _created;
            //millisecondsToDriveFor = (long)_millisecondsToDriveFor;
            //Debug.WriteLine("_ms (long): " + (long)_millisecondsToDriveFor);
            //Debug.WriteLine("ms (long)(long): " + millisecondsToDriveFor);
        }
    }
}
