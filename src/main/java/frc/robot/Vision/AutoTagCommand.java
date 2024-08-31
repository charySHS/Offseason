package frc.robot.Vision;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Limelight.LimelightHelpers;
import frc.robot.RobotContainer;

public class AutoTagCommand
{

    LimelightHelpers.LimelightTarget_Fiducial currentTarget;

    PIDController aimPID = new PIDController(0.165, 0, 0.01);
    PIDController xPID = new PIDController(2, 0, 0);
    PIDController yPID = new PIDController(2, 0, 0);

    boolean isShooting;
    boolean hasNote;

    public AutoTagCommand()
    {
        aimPID.setTolerance(5);
        yPID.setTolerance(0.025);
        xPID.setTolerance(0.08);
    }

//    @Override
//    public void initialize()
//    {
//        isShooting = false;
//        hasNote = false;
//    }
//
//    @Override
//    public void execute()
//    {
//        var name = "";
//
//        var target = Vision.GetBestTarget();
//        if (target == null)
//        {
//            var request = RobotContainer.Get().GetDefaultDriveRequest();
//            RobotContainer.Get().drivetrain.setControl(request);
//            return;
//        }
//    }
}
