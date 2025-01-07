package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;

public class MotorConstants {
    // Drive motor configuration
        public static final int driveMotorCurrentLimit = 50;
        public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 40;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    
}
