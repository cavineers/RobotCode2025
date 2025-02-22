package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class HealthCheck {
    private static void batteryVoltage() {
        // Get the battery voltage
        double voltage = RobotController.getBatteryVoltage();
        if (voltage < 12.0) {
            // Log a warning if the battery voltage is low
            if (voltage < 7.0) {
                Logger.recordOutput("Brownout", true);
            } 
            Logger.recordOutput("Voltage", voltage);
        }
        }


    private static void ifJoysticks() {
        // Get the battery voltage
        boolean joysticks =  DriverStation.isJoystickConnected(Constants.OIConstants.kDriverJoystickPort);
                   
        if(joysticks){
            // Log a warning if the joystick is not connected
            Logger.recordOutput("Joysticks connected", joysticks);
        }
    }

    private static void autoSelected(){
        // 
        boolean auto = DriverStation.isAutonomousEnabled();
        if (auto) {
            Logger.recordOutput("Auto Chosen", auto);
        }
    }

    public void motorFaults(){
        // 
        boolean hasfault = SparkBase.Faults.hasActiveFault();
        if (hasfault) {
            Logger.recordOutput("Auto Chosen", hasfault);
        }
    }    
}

