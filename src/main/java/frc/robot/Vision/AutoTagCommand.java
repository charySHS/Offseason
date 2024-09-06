package frc.robot.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.arm.ArmSubsystem;
import friarLib3.math.FriarMath;
import org.littletonrobotics.junction.Logger;

public class AutoTagCommand extends Command
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

    @Override
    public void initialize()
    {
        isShooting = false;
        hasNote = false;
    }

    @Override
    public void execute()
    {
        var name = "";

        var target = Vision.GetBestTarget();
        if (target == null)
        {
            var request = RobotContainer.Get().GetDefaultDriveRequest();
            RobotContainer.Get().drivetrain.setControl(request);
            return;
        }

        currentTarget = target;

        int id = (int)target.fiducialID;

        if (id == 4 || id ==7)
        {
            ExecuteSpeaker();
            return;
        }

        if (id == 5 || id ==6)
        {
            ExecuteAmp();
            return;
        }

        //        if (id == 1 || id == 2 || id == 9 || id == 10)
        if (id == 1 || id == 9)
        {
            ExecuteSource();
            return;
        }

        if (id >= 11)
        {
            ExecuteStage();
        }
    }

    private void ExecuteSpeaker()
    {
        double minDist = 1.52;
        double crossOver = 2.45;
        double maxDist = 4.28;

        double minArmRot = -0.058;
        double maxArmRot = ArmSubsystem.EArmPosition.Stowed.Rotations + 0.01;

        double minPivRot = ArmSubsystem.EPivotPosition.Stowed.Rotations;
        double maxPivRot = -0.265;

        // -- Auto Moving Arm
        Pose3d pose = currentTarget.getRobotPose_TargetSpace();
        var x = pose.getTranslation().getZ();
        var y = pose.getTranslation().getX();

        var dist = Math.sqrt((x * x) + (y * y));
        dist = MathUtil.clamp(dist, minDist, maxDist);

        Logger.recordOutput("AutoTag.Speaker.dist", dist);
        Logger.recordOutput("AutoTag.Speaker.y", y);

        var armPos = minArmRot;
        var pivPos = minPivRot;

        if (dist < crossOver)
        {
            armPos = FriarMath.Remap(dist, minDist, crossOver, minArmRot, maxArmRot, false);
        }
        else
        {
            pivPos = FriarMath.Remap(dist, crossOver, maxDist, minPivRot, maxPivRot, false);
        }

        Logger.recordOutput("AutoTag.Speaker.arm", armPos);
        Logger.recordOutput("AutoTag.Speaker.pivot", pivPos);

        // -- Auto Lineup
        var targetAngle = Math.toDegrees(Math.asin(-x / dist)) - 90;
        if (y < 0)
        {
            targetAngle *= -1;
        }

        var currentAngle = Math.toDegrees(pose.getRotation().getY());

        Logger.recordOutput("AutoTag.Speaker.theta", targetAngle);
        Logger.recordOutput("AutoTag.Speaker.robotAngle", currentAngle);

        var rotationRate = aimPID.calculate(currentAngle, targetAngle);

        var output = RobotContainer.Get().GetVelocityForThrottle();
        var request = RobotContainer.Get().driveFieldCentric
            .withVelocityX(output.getX())
            .withVelocityY(output.getY())
            .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);


        if (dist <= maxDist && !isShooting && aimPID.atSetpoint())
        {
            System.out.println("At setpoint");
            if (dist < crossOver)
            {
                System.out.println("Moving Arm");
                CommandScheduler.getInstance().schedule(
                    RobotContainer.Get().Arm.Command_GoToPosition(armPos)
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(RobotContainer.Get().Command_ScoreSpeaker())
                );
            }
            else if (dist == crossOver)
            {
                System.out.println("Shooting");
                CommandScheduler.getInstance().schedule(
                    RobotContainer.Get()
                                  .Command_ScoreSpeaker()
                );
            }
            else
            {
                System.out.println("Moving Pivot");
                CommandScheduler.getInstance().schedule(
                    RobotContainer.Get().Arm.Command_GoToPivotPosition(pivPos)
                        .andThen(Commands.waitSeconds(0.5))
                        .andThen(RobotContainer.Get().Command_ScoreSpeaker())
                );
            }

            isShooting = true;
        }
    }

    private void ExecuteAmp()
    {
        Pose3d pose = currentTarget.getRobotPose_TargetSpace();
        double posY = pose.getTranslation().getX();
        double posX = pose.getTranslation().getZ();
        double angleY = Math.toDegrees(pose.getRotation().getY());

        Logger.recordOutput("AutoTag.posX", posX);
        Logger.recordOutput("AutoTag.posY", posY);
        Logger.recordOutput("AutoTag.angleY", angleY);

        //double rotationalOffset = 0.05;
        var rotationRate = aimPID.calculate(angleY, 0);
        Logger.recordOutput("AutoTag.rotationRate", rotationRate);

        double offset = 0.05;

        double finalX = xPID.calculate(posX, -1.1);
        double finalY = yPID.calculate(posY, 0 + offset);

        var request = RobotContainer.Get().driveRobotCentric
            .withVelocityX(finalX)
            .withVelocityY(-finalY)
            .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);

        if (!isShooting && xPID.atSetpoint() && yPID.atSetpoint() && aimPID.atSetpoint())
        {
            CommandScheduler.getInstance()
                            .schedule(Commands.sequence(
                                RobotContainer.Get().Arm.Command_GoToPose(ArmSubsystem.EPose.Amp),
                                RobotContainer.Get().Command_DriveForward(
                                    1, .45),
                                Commands.waitSeconds(.5),
                                RobotContainer.Get().Command_ScoreAmp()
                            ));

            isShooting = true;
        }

    }

    private void ExecuteSource()
    {
        Pose3d pose = currentTarget.getRobotPose_TargetSpace();
        var posX = pose.getTranslation().getZ();
        var posY = pose.getTranslation().getX();
        var angleY = Math.toDegrees(pose.getRotation().getY());

        Logger.recordOutput("AutoTag.posX", posX);
        Logger.recordOutput("AutoTag.posY", posY);
        Logger.recordOutput("AutoTag.angleY", angleY);

        var rotationRate = aimPID.calculate(angleY, 0);
        Logger.recordOutput("AutoTag.rotationRate", rotationRate);

        double finalX = xPID.calculate(posX, -1.1);
        double finalY = yPID.calculate(posY, 0);

        var request = RobotContainer.Get().driveRobotCentric
            .withVelocityX(finalX)
            .withVelocityY(-finalY)
            .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);

        //        if (!IsShooting && !HasNote && XPID.atSetpoint() && YPID.atSetpoint())
        //        {
        //            CommandScheduler.getInstance().schedule(Commands.sequence(
        //                RobotContainer.Get().Command_DriveForward(-1, 1),
        //                Commands.waitSeconds(.5),
        //                RobotContainer.Get().Pose.Command_GoToPose(PoseManager.EPose.Source),
        //                RobotContainer.Get().Intake.Command_IntakeNote(true),
        //                RobotContainer.Get().Pose.Command_GoToPose(PoseManager.EPose.Stowed))
        //            );
        //            IsShooting = true;
        //        }
    }

    private void ExecuteStage()
    {
        Pose3d pose = currentTarget.getRobotPose_TargetSpace();
        var posX = pose.getTranslation().getZ();
        var posY = pose.getTranslation().getX();
        var angleY = Math.toDegrees(pose.getRotation().getY());

        Logger.recordOutput("AutoTag.posX", posX);
        Logger.recordOutput("AutoTag.posY", posY);
        Logger.recordOutput("AutoTag.angleY", angleY);

        var rotationRate = aimPID.calculate(angleY, 0);

        double offset = 0.05;

        double finalX = xPID.calculate(posX, -1);
        double finalY = yPID.calculate(posY, 0 + offset);

        var request = RobotContainer.Get().driveRobotCentric
            .withVelocityX(finalX)
            .withVelocityY(-finalY)
            .withRotationalRate(-rotationRate);

        RobotContainer.Get().drivetrain.setControl(request);

    }


}
