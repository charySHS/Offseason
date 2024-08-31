package frc.robot.Vision;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.Limelight.Limelight;
import frc.robot.Limelight.LimelightHelpers;
import org.littletonrobotics.junction.AutoLogOutput;

public class Vision
{
    static private final Set<Integer> ValidTags = Set.of(
        1, 2     // Blue Source
        , 4          // Red Speaker (center)
        , 5          // Red Amp
        , 6          // Blue Amp
        , 7          // Blue Speaker (center)
        , 9, 10      // Red Source
        , 11, 12, 13 // Red Stage
        , 14, 15, 16 // Red Stage
    );

    static private double limelightX = 0.0;
    static private double limelightY = 0.0;

    static private boolean ResultsAreStale = true;

    static private LimelightHelpers.Results LatestResults = null;

    static public void Periodic()
    {
        ResultsAreStale = true;
    }

    static private void UpdateResults()
    {
        if (ResultsAreStale)
        {
            limelightX = LimelightHelpers.getTX(VisionConstants.limelight);
            limelightY = LimelightHelpers.getTY(VisionConstants.limelight);

            LatestResults = LimelightHelpers.getLatestResults(VisionConstants.limelight).targetingResults;
            ResultsAreStale = false;
        }
    }

    static public LimelightHelpers.LimelightTarget_Fiducial GetBestTarget()
    {
        UpdateResults();

        LimelightHelpers.LimelightTarget_Fiducial bestTarget = null;

        // loop through all fid targets to find valid target that's closest
        for (LimelightHelpers.LimelightTarget_Fiducial target : LatestResults.targets_Fiducials)
        {
            if (ValidTags.contains((int)target.fiducialID))
            {
                if (bestTarget == null || Math.abs(target.tx) < Math.abs(bestTarget.tx))
                {
                    bestTarget = target;
                }
            }
        }

        return bestTarget;
    }

    static public LimelightHelpers.LimelightTarget_Detector GetBestNoteTarget()
    {
        LimelightHelpers.LimelightTarget_Detector bestNoteTarget = null;

        for (LimelightHelpers.LimelightTarget_Detector target : LatestResults.targets_Detector)
        {
            if (bestNoteTarget == null || Math.abs(target.tx) < Math.abs(bestNoteTarget.tx))
            {
                bestNoteTarget = target;
            }
        }
        return bestNoteTarget;
    }

    @AutoLogOutput
    static public double GetDistanceToNote()
    {
        return -(VisionConstants.limelightHeightInches - VisionConstants.noteHeightInches)
            / Math.tan(
                Units.degreesToRadians(
                    limelightY + VisionConstants.limelightAngleDegrees
                )
        );
    }

    public Pose2d GetNotePose(Pose2d robotPose)
    {
        return robotPose
            .transformBy(VisionConstants.robotToCam)
            .transformBy(
                new Transform2d(
                    new Translation2d(
                        Units.inchesToMeters(GetDistanceToNote()),
                        robotPose
                            .getRotation()
                            .plus(Rotation2d.fromDegrees(limelightX))
                    ),
                    robotPose
                        .getRotation()
                        .plus(Rotation2d.fromDegrees(limelightX))
                )
            );
    }

}
