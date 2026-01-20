package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Logger;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.AngularVelocity3d;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Drivetrain subsystem managing swerve drive and vision-based pose estimation.
 * Uses YAGSL for swerve control and YALL for Limelight vision integration.
 */
public class DrivetrainSubsystem extends SubsystemBase {
  
  private final SwerveDrive swerveDrive;
  private final Limelight limelight;
  private final LimelightPoseEstimator poseEstimator;
  
  public DrivetrainSubsystem() {
    // Initialize swerve drive from JSON configuration
    SwerveDrive tempDrive = null;
    try {
      tempDrive = new SwerveParser(DrivetrainConstants.SWERVE_JSON_FILE)
        .createSwerveDrive(
            DrivetrainConstants.MAX_SPEED.in(MetersPerSecond), 
            DrivetrainConstants.START_POSE
        );
      Logger.log("Drivetrain/InitSuccess", true);
    } catch (IOException exception) {
      Logger.logFault("Couldn't parse swerve json file.", AlertType.kError);
      Logger.log("Drivetrain/InitSuccess", false);
    }
    swerveDrive = tempDrive;

    // Configure telemetry verbosity
    SwerveDriveTelemetry.verbosity = DrivetrainConstants.TELEMETRY_VERBOSITY;

    // Initialize Limelight camera
    limelight = new Limelight(VisionConstants.LIMELIGHT_NAME);
    
    // Configure Limelight settings (only once in constructor for performance)
    limelight.getSettings()
             .withLimelightLEDMode(LEDMode.PipelineControl)
             .withCameraOffset(VisionConstants.CAMERA_OFFSET)
             .save();
             
    // Create MegaTag2 pose estimator for AprilTag-based localization
    poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    
    Logger.log("Vision/Initialized", true);
    Logger.log("Vision/CameraName", VisionConstants.LIMELIGHT_NAME);
  }

  @Override
  public void periodic() {
    if (swerveDrive == null) return;
    
    // Update robot orientation for MegaTag2 (CRITICAL for accurate pose estimation)
    updateRobotOrientation();
    
    // Process vision measurements and update odometry
    processVisionMeasurement();
    
    // Log current state
    Logger.log("Drivetrain/Pose", swerveDrive.getPose());
    Logger.log("Drivetrain/Velocity", swerveDrive.getFieldVelocity());
  }

  /**
   * Updates the Limelight with current robot orientation and angular velocities.
   * This is REQUIRED for MegaTag2 to properly compensate for robot motion.
   */
  private void updateRobotOrientation() {
    ChassisSpeeds robotVelocity = swerveDrive.getRobotVelocity();
    
    limelight.getSettings()
             .withRobotOrientation(new Orientation3d(
                 swerveDrive.getGyroRotation3d(),
                 VisionConstants.getAngularVelocity3d(robotVelocity.omegaRadiansPerSecond)
             ));
    // Note: No .save() here - settings were saved in constructor to reduce NT traffic
  }

  /**
   * Processes vision measurements from Limelight and updates odometry if valid.
   * Implements multiple safety checks to ensure pose estimate quality.
   */
  private void processVisionMeasurement() {
    Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
    
    // Early exit if no vision data available
    if (visionEstimate.isEmpty()) {
      Logger.log("Vision/HasTarget", false);
      return;
    }
    
    PoseEstimate estimate = visionEstimate.get();
    
    // Log raw vision data for analysis and tuning
    logVisionData(estimate);
    
    // Get current robot velocity for motion blur check
    ChassisSpeeds robotVelocity = swerveDrive.getRobotVelocity();
    double angularVelocityDegPerSec = Math.abs(Math.toDegrees(robotVelocity.omegaRadiansPerSecond));
    
    // Safety check: Reject if spinning too fast (prevents motion blur issues)
    if (angularVelocityDegPerSec > VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
      rejectVisionMeasurement("High angular velocity: " + String.format("%.1f°/s", angularVelocityDegPerSec));
      return;
    }
    
    // Validate vision estimate quality
    String rejectionReason = getVisionRejectionReason(estimate);
    if (rejectionReason != null) {
      rejectVisionMeasurement(rejectionReason);
      return;
    }
    
    // Vision estimate passed all checks - add to odometry
    addVisionMeasurementToOdometry(estimate);
    
    Logger.log("Vision/Rejected", false);
    Logger.log("Vision/AcceptedPose", estimate.pose.toPose2d());
  }

  /**
   * Logs all relevant vision data for debugging and analysis.
   */
  private void logVisionData(PoseEstimate estimate) {
    Logger.log("Vision/HasTarget", true);
    Logger.log("Vision/TagCount", estimate.tagCount);
    Logger.log("Vision/AvgDistance", estimate.avgTagDist);
    Logger.log("Vision/MinAmbiguity", estimate.getMinTagAmbiguity());
    Logger.log("Vision/Timestamp", estimate.timestampSeconds);
    Logger.log("Vision/RawPose", estimate.pose.toPose2d());
    Logger.log("Vision/Latency", estimate.latency);
  }

  /**
   * Logs rejection of vision measurement with reason.
   */
  private void rejectVisionMeasurement(String reason) {
    Logger.log("Vision/Rejected", true);
    Logger.log("Vision/RejectReason", reason);
  }

  /**
   * Validates vision estimate against quality thresholds.
   * @return Rejection reason if invalid, null if valid
   */
  private String getVisionRejectionReason(PoseEstimate estimate) {
    if (estimate.tagCount < VisionConstants.MIN_TAG_COUNT) {
      return "No tags visible";
    }
    
    if (estimate.avgTagDist > VisionConstants.MAX_TAG_DISTANCE_METERS) {
      return String.format("Tags too far: %.2fm (max: %.2fm)", 
          estimate.avgTagDist, VisionConstants.MAX_TAG_DISTANCE_METERS);
    }
    
    if (estimate.getMinTagAmbiguity() > VisionConstants.MAX_TAG_AMBIGUITY) {
      return String.format("High ambiguity: %.2f (max: %.2f)", 
          estimate.getMinTagAmbiguity(), VisionConstants.MAX_TAG_AMBIGUITY);
    }
    
    return null; // Valid - passed all checks
  }

  /**
   * Adds validated vision measurement to YAGSL's pose estimator.
   * Uses adaptive standard deviations based on measurement quality.
   */
  private void addVisionMeasurementToOdometry(PoseEstimate estimate) {
    // Select appropriate standard deviations based on number of tags
    // More tags = higher confidence = lower standard deviation
    double[] baseStdDevs = (estimate.tagCount >= 2) 
        ? VisionConstants.VISION_STD_DEVS_MULTI_TAG.clone()
        : VisionConstants.VISION_STD_DEVS_SINGLE_TAG.clone();
    
    // Further adjust based on distance (closer tags = more accurate)
    // Use a safer multiplier that doesn't grow too large
    double distanceMultiplier = Math.min(Math.pow(estimate.avgTagDist, 2) / 4.0, 5.0);
    baseStdDevs[0] *= distanceMultiplier;
    baseStdDevs[1] *= distanceMultiplier;
    
    // YAGSL's addVisionMeasurement handles the SwerveDrivePoseEstimator internally
    swerveDrive.addVisionMeasurement(
        estimate.pose.toPose2d(),
        estimate.timestampSeconds,
        VecBuilder.fill(baseStdDevs[0], baseStdDevs[1], baseStdDevs[2])
    );
    
    // Log applied standard deviations for tuning
    Logger.log("Vision/StdDevX", baseStdDevs[0]);
    Logger.log("Vision/StdDevY", baseStdDevs[1]);
    Logger.log("Vision/DistanceMultiplier", distanceMultiplier);
  }

  /**
   * Drive the robot field-relative.
   * @param xSpeed Forward speed (m/s, positive = forward)
   * @param ySpeed Strafe speed (m/s, positive = left)
   * @param rSpeed Rotation speed (rad/s, positive = CCW)
   */
  public void driveFieldRelative(double xSpeed, double ySpeed, double rSpeed) {
    if (swerveDrive == null) return;
    
    swerveDrive.drive(
        new Translation2d(xSpeed, ySpeed), 
        rSpeed, 
        true,   // Field relative
        false   // Not open loop
    );
  }

  /**
   * Drive the robot robot-relative.
   * @param xSpeed Forward speed (m/s, positive = forward)
   * @param ySpeed Strafe speed (m/s, positive = left)
   * @param rSpeed Rotation speed (rad/s, positive = CCW)
   */
  public void driveRobotRelative(double xSpeed, double ySpeed, double rSpeed) {
    if (swerveDrive == null) return;
    
    swerveDrive.drive(
        new Translation2d(xSpeed, ySpeed), 
        rSpeed, 
        false,  // Robot relative
        false   // Not open loop
    );
  }

  /**
   * Get current robot pose from odometry.
   * @return Current pose on field
   */
  public Pose2d getPose() {
    return swerveDrive != null ? swerveDrive.getPose() : new Pose2d();
  }

  /**
   * Reset robot pose (useful for auto initialization).
   * @param pose New pose to set
   */
  public void resetPose(Pose2d pose) {
    if (swerveDrive != null) {
      swerveDrive.resetOdometry(pose);
      Logger.log("Drivetrain/PoseReset", true);
    }
  }

  /**
   * Zero the gyroscope heading.
   */
  public void zeroGyro() {
    if (swerveDrive != null) {
      swerveDrive.zeroGyro();
      Logger.log("Drivetrain/GyroZeroed", true);
    }
  }

  /**
   * Lock swerve modules in X formation to prevent movement.
   */
  public void lockWheels() {
    if (swerveDrive != null) {
      swerveDrive.lockPose();
    }
  }

  /**
   * Get the current velocity of the robot.
   * @return Robot velocity
   */
  public ChassisSpeeds getVelocity() {
    return swerveDrive != null ? swerveDrive.getRobotVelocity() : new ChassisSpeeds();
  }
}