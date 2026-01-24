package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
              DrivetrainConstants.START_POSE);
    } catch (IOException exception) {
      Logger.logFault("Couldn't parse swerve json file.", AlertType.kError);
    }

    SwerveDriveTelemetry.verbosity = DrivetrainConstants.TELEMETRY_VERBOSITY;
  }

  @Override
  public void periodic() {

  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rSpeed) {
    swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rSpeed, true, false);
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rSpeed) {
    if (swerveDrive == null)
      return;

    swerveDrive.drive(
        new Translation2d(xSpeed, ySpeed),
        rSpeed,
        false, // Robot relative
        false);
  }

  public Pose2d getPose() {
    return swerveDrive != null ? swerveDrive.getPose() : new Pose2d();
  }

  public void resetPose(Pose2d pose) {
    if (swerveDrive != null) {
      swerveDrive.resetOdometry(pose);
      Logger.log("Drivetrain/PoseReset", true);
    }
  }

  public void zeroGyro() {
    if (swerveDrive != null) {
      swerveDrive.zeroGyro();
      Logger.log("Drivetrain/GyroZeroed", true);
    }
  }

  public void lockWheels() {
    if (swerveDrive != null) {
      swerveDrive.lockPose();
    }
  }

  public void driveToPose(Translation2d pose, Rotation2d rotation) {
    if (swerveDrive != null) {
      swerveDrive.drive(pose, rotation.getDegrees(), false, false);
    }
  }

  public ChassisSpeeds getVelocity() {
    return swerveDrive != null ? swerveDrive.getRobotVelocity() : new ChassisSpeeds();
  }
}
