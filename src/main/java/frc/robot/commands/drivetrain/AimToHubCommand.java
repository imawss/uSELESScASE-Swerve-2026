package frc.robot.commands.drivetrain;

import frc.robot.Logger;
import frc.robot.constants.AutonomousConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class AimToHubCommand extends Command {

    Pose2d robotPose; 
    DrivetrainSubsystem drivetrain;
    double maxAngularRate;
    Translation2d hubCenter = AutonomousConstants.BLUE_HUB_CENTER; 
    Translation2d robotToHub;
    Rotation2d angleToHub;
    Alliance currentAlliance;
    PIDController rotationController;

    public AimToHubCommand(DrivetrainSubsystem _drivetrain) {
        drivetrain = _drivetrain;
        rotationController = new PIDController(AutonomousConstants.kP_rot, 0.05f, AutonomousConstants.kD_rot);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Logger.log("Commands/AimToHub/Running", true);
        Logger.log("Commands/AimToHub/HubCenterX", hubCenter.getX());
        Logger.log("Commands/AimToHub/HubCenterY", hubCenter.getY());
    }

     @Override
    public void execute() {
        robotPose = drivetrain.getPose();

        currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
        //hubCenter = (currentAlliance == Alliance.Blue) ? AutonomousConstants.BLUE_HUB_CENTER : AutonomousConstants.RED_HUB_CENTER;
        
        robotToHub = hubCenter.minus(robotPose.getTranslation());
        angleToHub = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        drivetrain.driveToPose(robotToHub, angleToHub);
    }

     @Override
    public void end(boolean interrupted) {
        drivetrain.lockWheels();
        Logger.log("Commands/AimToHub/Running", false);
    }

     @Override
    public boolean isFinished() {
        return false;
    }
}
