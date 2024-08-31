package frc.robot.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class VisionConstants
{
    public static final String limelight = "limelight-main";

    public static final String driverCam = "dash-cam";

    public static final double limelightAngleDegrees = 1; // TODO: TUNE!!!
    public static final double limelightHeightInches = 1; // TODO: TUNE!!!
    public static final double noteHeightInches = 2; // TODO: Verify

    public static final Transform2d robotToCam =
        new Transform2d(
            new Translation2d(0, 0), new Rotation2d(0)
        );
}
