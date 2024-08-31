// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Limelight.Limelight;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.swerve.Requests.SwerveFieldCentricFacingAngle;
import frc.robot.Subsystems.swerve.Swerve;
import frc.robot.Subsystems.swerve.SwerveConstants;

import frc.robot.Subsystems.swerve.SwerveTelemetry;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionConstants;
import friarLib3.commands.CommandQueue;
import friarLib3.general.XboxStalker;
import friarLib3.math.LookupTable;
import org.littletonrobotics.junction.Logger;

public class RobotContainer
{
  private SendableChooser<Command> autoChooser;

  /* Controllers */
  public static CommandXboxController Driver = new CommandXboxController(0);
  public static CommandXboxController Operator = new CommandXboxController(1);
  private final CommandXboxController test = new CommandXboxController(2);

  static private RobotContainer _This;
  static public RobotContainer Get() { return _This; }

  static public final PowerDistribution PDH = new PowerDistribution();

  // --------------------------------------------------------------------------------------------
  // -- Misc Input
  // --------------------------------------------------------------------------------------------
  static private final DigitalInput DIO_Zero = new DigitalInput(9);
  static private final Trigger Trigger_Zero = new Trigger(() -> DIO_Zero.get());

  // --------------------------------------------------------------------------------------------
  // -- Tuning Values
  // --------------------------------------------------------------------------------------------
  private final double MaxRotationsPerSecond = 0.75;
  private LookupTable ThrottleLut = new LookupTable.Normalized()
      .AddValue(0.1, 0.0) // dead-band
      .AddValue(0.35, 0.05)
      .AddValue(0.75, 0.2);

  private LookupTable TurnLut = new LookupTable.Normalized()
      .AddValue(0.1, 0.0) // dead-band
      .AddValue(0.35, 0.05)
      .AddValue(0.75, 0.2);


  private double MaxSpeed =
      SwerveConstants.kSpeedAt12VoltsMps;

  private double MaxAngularRate = 1.5 * Math.PI;

  // --------------------------------------------------------------------------------------------
  // -- Subsystems
  // --------------------------------------------------------------------------------------------
  private boolean isRed;

  public  Swerve drivetrain = SwerveConstants.Drivetrain;

  public ArmSubsystem Arm;
  public IntakeSubsystem Intake;

  public CommandQueue commandQueue;

  public Vision vision;

  // --------------------------------------------------------------------------------------------
  // -- Drive requests
  // --------------------------------------------------------------------------------------------
  public final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * Constants.Drive.StickDeadband)
      .withRotationalDeadband(MaxAngularRate * Constants.Drive.RotationalDeadband)
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  private final SwerveFieldCentricFacingAngle driveTowardsAngle = new SwerveFieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withHeadingController(SwerveConstants.azimuthController)
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
  public final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  public final SwerveRequest.RobotCentric AimRobot = new SwerveRequest.RobotCentric().withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

  private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(MaxSpeed);

  private boolean RotationModeIsRobotCentric = false;


  public RobotContainer()
  {
    _This = this;

    // Cancel any previous commands
    CommandScheduler.getInstance().cancelAll();

    commandQueue = new CommandQueue();
    vision = new Vision();

    // Setup subsystems and bindings

    test.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    test.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    test.leftTrigger()
        .onTrue(
            drivetrain.applyRequest(() -> driveTowardsAngle.withTargetDirection(Rotation2d.fromDegrees(180)))
        );

    test.y().whileTrue(drivetrain.SysIDQuasistatic(SysIdRoutine.Direction.kForward));
    test.a().whileTrue(drivetrain.SysIDQuasistatic(SysIdRoutine.Direction.kReverse));
    test.b().whileTrue(drivetrain.SysIDDynamic(SysIdRoutine.Direction.kForward));
    test.x().whileTrue(drivetrain.SysIDDynamic(SysIdRoutine.Direction.kReverse));

  }

  // -----------------------------------------------------------------------------------------------
  // -- Commands
  // -----------------------------------------------------------------------------------------------

  public Command Command_RumbleControllers()
  {
    return Commands.runOnce(() ->
        CommandScheduler.getInstance().schedule(
            Commands.sequence(
                Commands.waitSeconds(0.5),
                Commands.runOnce(() ->
                                 {
                                   Operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
                                   Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
                                 }),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() ->
                                 {
                                   Operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                                   Driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
                                 })
            )
        )
    );
  }

  public void SetAllianceColor(boolean color)
  {
    isRed = color;
  }

  private void ConfigureChecks()
  {
    if (DriverStation.isFMSAttached())
    {
      if (!Constants.FeatureFlags.SwerveEnabled)
      {
        System.out.println("Swerve is disabled, but the robot is FMS-attached. This shouldn't happen!");
        DriverStation.reportError(
            "Swerve is Disabled, but the robot is FMS-connected. This shouldn't happen", false
        );

      }
      if (Constants.FeatureFlags.DebugEnabled)
      {
        System.err.println("Robot is FMS-connected & debug mode is enabled. Recommended to not proceed");
        DriverStation.reportWarning("Robot is FMS-attached & debug mode is enabled. Recommended to not proceed", false);
      }
    }
    else {

    }
  }

  private void ConfigureMiscBindings()
  {
    Trigger_Zero.onTrue(
        Commands.sequence(
            Arm.Command_ZeroPivotEncoder(),
            Arm.Command_ZeroArmEncoder(),
            drivetrain.runOnce(drivetrain::seedFieldRelative),
            Commands.print("Zeroed!"),
            Commands.runOnce(() -> Limelight.setLEDMode_ForceBlink(VisionConstants.limelight)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> Limelight.setLEDMode_ForceOff(VisionConstants.limelight))
        )
            .unless(() -> DriverStation.isEnabled())
            .ignoringDisable(Constants.FeatureFlags.IgnoringDisabled)
    );
  }

  public Command GetAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void RunPitTestRoutine()
  {
    // Command pitRoutine = new PitRoutine();
    // pitRoutine.schedule();
  }

  public void periodic(double dt)
  {
    XboxStalker.stalk(Driver, Operator);
    Logger.recordOutput("Note Pose", vision.GetNotePose(drivetrain.getState().Pose));
  }

}
