package frc.robot.commands.auto;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class AutoHelpers {

    public static Command clearOverrides() {
        return Commands.runOnce(() -> {
            PPHolonomicDriveController.clearFeedbackOverrides();
        });
    }

}
