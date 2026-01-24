package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class AutonomousConstants {
    
    //FIELD CONSTANTS
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.552, 4.021);
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.961, 4.021);

    //Aim To Hub Rotation PID
    public static final float kP_rot = 0.001f;
    public static final float kD_rot = 0.32f;

}
