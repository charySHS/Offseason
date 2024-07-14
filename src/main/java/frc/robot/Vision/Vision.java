package frc.robot.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Set;

import frc.robot.Limelight.Limelight;
import frc.robot.Limelight.LimelightHelpers;
import org.littletonrobotics.junction.AutoLogOutput;

public class Vision extends SubsystemBase {

    static private final Set<Integer> ValidTags = Set.of
    (
    1, 2           // Blue Source
    ,4             // Red Speaker (center)
    ,5             // Red Amp
    ,6             // Blue Amp
    ,7             // Blue Speaker (center)
    ,9 , 10        // Red Source
    ,11, 12, 13    // Red Stage
    ,14, 15, 16    // Blue Stage
    );

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
            LatestResults = Limelight.getLatestResults("").targetingResults;
            ResultsAreStale = false;
        }
    }

    static public LimelightHelpers.LimelightTarget_Fiducial GetBestTarget()
    {
        UpdateResults();

        LimelightHelpers.LimelightTarget_Fiducial bestTarget = null;

        // -- Loop through all fid targets to find valid, closest target
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

    static public LimelightHelpers.LimelightTarget_Detector GetBestNote()
    {
        LimelightHelpers.LimelightTarget_Detector bestNote = null;

        for (LimelightHelpers.LimelightTarget_Detector target : LatestResults.targets_Detector)
        {
            if (bestNote == null || Math.abs(target.tx) < Math.abs(bestNote.tx))
            {
                bestNote = target;
            }
        }

        return bestNote;
    }

    public static double ReturnDistance()
    {
        var target = GetBestTarget();
        if (target == null) { return 0; }

        return target.getRobotPose_TargetSpace().getTranslation().getZ();
    }
}
