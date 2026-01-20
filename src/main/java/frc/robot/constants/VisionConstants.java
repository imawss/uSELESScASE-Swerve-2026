package frc.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import limelight.networktables.AngularVelocity3d;

/**
 * Constants for vision-based pose estimation using Limelight.
 * Tune these values based on your robot's physical configuration and field testing.
 */
public final class VisionConstants {
  
  // ==================== LIMELIGHT CONFIGURATION ====================
  
  /**
   * Network Tables name of the Limelight camera.
   * Change this if you've renamed your Limelight in its web interface.
   */
  public static final String LIMELIGHT_NAME = "limelight";
  
  /**
   * Physical offset of the Limelight camera from the robot's center.
   * Coordinate system:
   * - X: forward/backward from center (positive = forward)
   * - Y: left/right from center (positive = left) 
   * - Z: up/down from center (positive = up)
   * - Rotation: camera tilt (roll, pitch, yaw in radians)
   * 
   * IMPORTANT: Measure these values accurately for best pose estimation!
   * 
   * Example measurements:
   * - Camera is 10 inches forward of robot center
   * - Camera is 2 inches left of robot center
   * - Camera is 24 inches above the ground
   * - Camera is tilted back 25 degrees from horizontal
   */
  public static final Pose3d CAMERA_OFFSET = new Pose3d(
      Inches.of(10).in(Meters),           // X offset (forward)
      Inches.of(2).in(Meters),            // Y offset (left)
      Inches.of(24).in(Meters),           // Z offset (height)
      new Rotation3d(
          0,                              // Roll (rotation around X axis)
          Math.toRadians(-25),            // Pitch (rotation around Y axis) - negative = tilted back
          0                               // Yaw (rotation around Z axis)
      )
  );
  
  // ==================== VISION FILTERING THRESHOLDS ====================
  
  /**
   * Maximum distance to AprilTags (in meters) for accepting vision measurements.
   * Tags farther than this are rejected because accuracy decreases with distance.
   * 
   * Recommended: 4.0 - 5.0 meters
   * Tune based on your camera quality and lighting conditions.
   */
  public static final double MAX_TAG_DISTANCE_METERS = 4.0;
  
  /**
   * Maximum ambiguity value for accepting vision measurements.
   * Ambiguity represents how certain the Limelight is about tag detection.
   * Lower values = more certain.
   * 
   * Range: 0.0 (perfect) to 1.0 (completely ambiguous)
   * Recommended: 0.2 - 0.3
   */
  public static final double MAX_TAG_AMBIGUITY = 0.3;
  
  /**
   * Maximum robot angular velocity (degrees/second) for accepting vision measurements.
   * When spinning faster than this, vision is rejected to prevent motion blur.
   * 
   * Recommended: 360 - 540 degrees/second
   * Lower values = more conservative (fewer rejections due to motion blur)
   */
  public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0;
  
  /**
   * Minimum number of AprilTags required to accept a vision measurement.
   * 
   * Recommended: 1 (accept single-tag measurements)
   * Set to 2 for more conservative filtering (only multi-tag measurements)
   */
  public static final int MIN_TAG_COUNT = 1;
  
  // ==================== VISION STANDARD DEVIATIONS ====================
  
  /**
   * Standard deviations for vision measurements with a SINGLE AprilTag visible.
   * Format: [X std dev, Y std dev, Rotation std dev]
   * 
   * Lower values = more trust in vision
   * Higher values = less trust in vision
   * 
   * Rotation is set very high (9999999) because we trust the gyro more for heading.
   * 
   * Recommended starting values: [1.0, 1.0, 9999999]
   * Tune X and Y based on observed vision accuracy during testing.
   */
  public static final double[] VISION_STD_DEVS_SINGLE_TAG = {1.0, 1.0, 9999999};
  
  /**
   * Standard deviations for vision measurements with MULTIPLE AprilTags visible.
   * Multi-tag measurements are more accurate, so we trust them more (lower values).
   * 
   * Recommended starting values: [0.5, 0.5, 9999999]
   * Should be lower than VISION_STD_DEVS_SINGLE_TAG.
   */
  public static final double[] VISION_STD_DEVS_MULTI_TAG = {0.5, 0.5, 9999999};
  
  // ==================== ROBOT ORIENTATION (FOR MEGATAG2) ====================
  
  /**
   * Pitch angular velocity for robot orientation updates.
   * Set to 0 if your gyro doesn't provide pitch rate.
   * 
   * If your gyro provides pitch rate, you can update this dynamically in the subsystem.
   */
  public static final double PITCH_RATE_DEG_PER_SEC = 0.0;
  
  /**
   * Roll angular velocity for robot orientation updates.
   * Set to 0 if your gyro doesn't provide roll rate.
   * 
   * If your gyro provides roll rate, you can update this dynamically in the subsystem.
   */
  public static final double ROLL_RATE_DEG_PER_SEC = 0.0;
  
  /**
   * Create AngularVelocity3d from robot's current angular velocities.
   * AngularVelocity3d constructor takes three separate AngularVelocity objects (pitch, roll, yaw).
   * 
   * @param omegaRadiansPerSecond Robot's yaw rate from chassis speeds
   * @return Complete AngularVelocity3d for YALL
   */
  public static AngularVelocity3d getAngularVelocity3d(double omegaRadiansPerSecond) {
    return new AngularVelocity3d(
        DegreesPerSecond.of(PITCH_RATE_DEG_PER_SEC),           // Pitch rate
        DegreesPerSecond.of(ROLL_RATE_DEG_PER_SEC),            // Roll rate
        DegreesPerSecond.of(Math.toDegrees(omegaRadiansPerSecond))  // Yaw rate
    );
  }
  
  // ==================== TUNING GUIDELINES ====================
  
  /*
   * TUNING PROCESS:
   * 
   * 1. CAMERA OFFSET:
   *    - Measure physical location precisely with a tape measure
   *    - Use CAD model if available
   *    - Verify pitch angle with a protractor or level app
   * 
   * 2. FILTERING THRESHOLDS:
   *    - Start with recommended values
   *    - Enable logging and observe rejection reasons
   *    - If too many valid measurements rejected: increase thresholds
   *    - If bad measurements accepted: decrease thresholds
   * 
   * 3. STANDARD DEVIATIONS:
   *    - Start with recommended values
   *    - Drive in a straight line and observe odometry vs vision
   *    - If vision pulls odometry too much: increase std devs
   *    - If vision doesn't correct odometry enough: decrease std devs
   *    - Log vision pose vs odometry pose to analyze accuracy
   * 
   * 4. FIELD TESTING:
   *    - Test with different lighting conditions
   *    - Test at various distances from tags
   *    - Test while spinning at different speeds
   *    - Observe pose estimation stability during match play
   * 
   * COMMON ISSUES:
   * 
   * - Odometry jumps when seeing tags:
   *   -> Increase standard deviations
   *   -> Check camera offset is accurate
   *   -> Verify camera is securely mounted (no vibration)
   * 
   * - Odometry doesn't correct from vision:
   *   -> Decrease standard deviations
   *   -> Check rejection logs - may be rejecting all measurements
   *   -> Verify Limelight can see AprilTags (check in web interface)
   * 
   * - Poor performance at competition:
   *   -> Different lighting - may need to adjust ambiguity threshold
   *   -> Verify field map is loaded correctly in Limelight
   *   -> Check Limelight firmware is up to date
   */
  
  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}