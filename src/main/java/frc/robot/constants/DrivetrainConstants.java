package frc.robot.constants;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivetrainConstants {
  
  public static final File SWERVE_JSON_FILE = new File(Filesystem.getDeployDirectory(), "swerve");

  public static final LinearVelocity MAX_SPEED = Units.MetersPerSecond.of(4.5);

  public static final Pose2d START_POSE = new Pose2d();

  public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.HIGH;
}
