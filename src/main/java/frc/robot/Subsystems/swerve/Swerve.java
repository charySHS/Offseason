package frc.robot.Subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.choreo.lib.*;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
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
import frc.robot.Vision.Vision;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem
{
    LoggableInputs AutoLog;

    private final boolean disabled;

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

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();

    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // Use for testing
    private SysIdRoutine SysIDRoutineTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this
            )
        );

    private final SysIdRoutine SysIDRoutineRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> SignalLogger.writeString("State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this
            )
        );

    private final SysIdRoutine SysIDRoutineSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                (state) -> SignalLogger.writeString("State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this
            )
        );

    // TODO: Change this line to the routine to test
    private final SysIdRoutine RoutineToApply = SysIDRoutineSteer;


    public Swerve(boolean disabled, SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.disabled = disabled;
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public Swerve(boolean disabled, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        this.disabled = disabled;
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void setControl(SwerveRequest request)
    {
        if (!disabled)
        {
            super.setControl(request);
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

    public Command pathFindToNote(Vision vision)
    {
        PathConstraints constraints =
            new PathConstraints(
                SwerveConstants.kSpeedAt12VoltsMps -1,
                4,
                edu.wpi.first.math.util.Units.degreesToRadians(450),
                edu.wpi.first.math.util.Units.degreesToRadians(540)
            );

        return AutoBuilder.pathfindToPose(
            // current pose, path constraints, rotation
            vision.GetNotePose(this.getState().Pose), constraints, 1, 0.0);
    }

    public Command rotationTest()
    {
        PathConstraints constraints =
            new PathConstraints(
                SwerveConstants.kSpeedAt12VoltsMps -1,
                4,
                edu.wpi.first.math.util.Units.degreesToRadians(450),
                edu.wpi.first.math.util.Units.degreesToRadians(540)
            );

        return AutoBuilder.pathfindToPose(
            this.getState()
                .Pose
                .rotateBy(Rotation2d.fromDegrees(90))
                .plus(new Transform2d(new Translation2d(2, 1), Rotation2d.fromDegrees(0))),
            constraints,
            0,
            0.0
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command runChoreoTrajectory(ChoreoTrajectory trajectory)
    {
        return Choreo.choreoSwerveCommand(
            trajectory,
            () -> (this.getState().Pose),
            SwerveConstants.choreoTranslationController,
            SwerveConstants.choreoTranslationController,
            SwerveConstants.choreoRotationController,
            ((ChassisSpeeds speeds) ->
                this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)
        )),
            () ->
            {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this);
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

    public Command SysIDQuasistatic(SysIdRoutine.Direction direction)
    {
        return RoutineToApply.quasistatic(direction);
    }

    public Command SysIDDynamic(SysIdRoutine.Direction direction)
    {
        return RoutineToApply.dynamic(direction);
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