package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final class OIConstants {
        public static final int kDriverJoystickPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }

    public static final class DIO {
        }

    public static final class CanIDs {

        public static final int LeftIntakeCanID = 1; //subject to change to upper/lower 
        public static final int RightIntakeCanID = 2; //changability in id numbers 
    }

    public static final class Intake {

        public static final double LeftIntakeForwardSpeed = 0.5; //change to actual speeds
        public static final double RightIntakeForwardSpeed = 0.5;
        public static final double LeftIntakeBackwardSpeed = 0.5;
        public static final double RightIntakeBackwardSpeed = 0.5;
        

    }
    public static final Mode simMode = Mode.SIM; 
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
