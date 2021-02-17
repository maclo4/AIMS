using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MistyRobotics.SDK.Events;

namespace MistyCSharpSkill2
{
    class MoveCommand
    {
        public long millisecondsToDriveFor { get; }
        public IDriveEncoderEvent driveEncoderData { get; }
        public MoveCommand(IDriveEncoderEvent _driveEncoderData, double _millisecondsToDriveFor )
        {
            driveEncoderData = _driveEncoderData;
            long milliSecondsToDriveFor = (long)_millisecondsToDriveFor;
        }
    }
}
