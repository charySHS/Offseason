package frc.robot.Subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Limelight.Limelight;
import frc.robot.Subsystems.SwerveConstants;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem
{
    LoggableInputs AutoLog;

    private static final double SimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);

    private final Rotation2d RedAllianceperspectiveRotation = Rotation2d.fromDegrees(180);

    private boolean hasAppliedOperatorPerspective = false;

    enum EDirection
    {
        Forward,
        Backward,
        Right,
        Left
    }

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0.49;
        for (var moduleLocation : m_moduleLocations)
        {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose,
            this::seedFieldRelative,
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
            new HolonomicPathFollowerConfig(new PIDConstants(3, 0, 0),
                                            new PIDConstants(3, 0, 0),
                                            SwerveConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
            this);


    }

    public void updateVision()
    {
        boolean useMegaTag2 = true;
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false)
        {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
            {
                if (mt1.rawFiducials[0].ambiguity > 0.7) { doRejectUpdate = true; }
                if (mt1.rawFiducials[0].distToCamera > 3) { doRejectUpdate = true; }
            }

            if (mt1.tagCount == 0) { doRejectUpdate = true; }

            if (!doRejectUpdate)
            {
                this.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
                this.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }

        }
        else
        {
            LimelightHelpers.SetRobotOrientation(
                "limelight",
                this.m_odometry.getEstimatedPosition().getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0);

            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (Math.abs(this.getPigeon2().getRate()) > 720) { doRejectUpdate = true; }

            if (mt2.tagCount == 0) { doRejectUpdate = true; }

            if (!doRejectUpdate)
            {
                this.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                this.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }



    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(SimLoopPeriod);
    }

    @Override
    public void periodic()
    {
        Logger.processInputs(this.getClass().getSimpleName(), AutoLog);

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled())
        {
            DriverStation.getAlliance()
                .ifPresent(
                    (allianceColor) ->
                    {
                        this.setOperatorPerspectiveForward(
                            allianceColor == DriverStation.Alliance.Red
                            ? RedAllianceperspectiveRotation
                                : BlueAlliancePerspectiveRotation
                        );
                        hasAppliedOperatorPerspective = true;
                    }
                );
            updateVision();
        }
    }

}