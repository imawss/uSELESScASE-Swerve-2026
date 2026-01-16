package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Logger;
import frc.robot.constants.DrivetrainConstants;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private SwerveDrive swerveDrive;
  
  public DrivetrainSubsystem() {
    try {
      this.swerveDrive = new SwerveParser(DrivetrainConstants.SWERVE_JSON_FILE)
        .createSwerveDrive(DrivetrainConstants.MAX_SPEED.in(MetersPerSecond), 
        DrivetrainConstants.START_POSE
      );
    } catch (IOException exception) {
      Logger.logFault("Couldn't parse swerve json file.", AlertType.kError);
    } 

    SwerveDriveTelemetry.verbosity = DrivetrainConstants.TELEMETRY_VERBOSITY;
  }

  @Override
  public void periodic() {
  
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rSpeed) {

    // Open loopun ne etkisi olur bilmiyom deneyin
    swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rSpeed, true, false);
  }

}
