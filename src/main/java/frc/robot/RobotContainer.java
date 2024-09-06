// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.Subsystems.arm.ArmSubsystem;
import frc.robot.Subsystems.intake.IntakeSubsystem;
import frc.robot.Subsystems.swerve.Requests.SwerveFieldCentricFacingAngle;
import frc.robot.Subsystems.swerve.Swerve;
import frc.robot.Subsystems.swerve.SwerveConstants;

import frc.robot.Subsystems.swerve.SwerveTelemetry;
import frc.robot.Vision.AutoTagCommand;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionConstants;
import friarLib3.commands.CommandQueue;
import friarLib3.general.XboxStalker;
import friarLib3.math.LookupTable;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

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
  static private final Trigger Trigger_Zero = new Trigger(DIO_Zero::get);

  // --------------------------------------------------------------------------------------------
  // -- Tuning Values
  // --------------------------------------------------------------------------------------------
  private final double MaxRotationsPerSecond = 0.75;
  private LookupTable ThrottleLUT = new LookupTable.Normalized()
      .AddValue(0.1, 0.0) // dead-band
      .AddValue(0.35, 0.05)
      .AddValue(0.75, 0.2);

  private LookupTable TurnLUT = new LookupTable.Normalized()
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
    ConfigureAutoCommands();

    SetDefaultCommands();
    SetAllianceColor(!isRed);

    ConfigureMiscBindings();
    ConfigureDriverBindings();
    ConfigureOperatorBindings();
    ConfigureChecks();

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

  // -----------------------------------------------------------------------------
  // -- Driver Binds
  // -----------------------------------------------------------------------------

  private void ConfigureDriverBindings()
  {

    // Slow drive, relative to bot pose
    Driver.povUp().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5))); // Fine-tune control forwards
    Driver.povDown().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5))); // Fine-tune control backwards
    Driver.povRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityY(-.5))); // Fine-tune control right
    Driver.povLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityY(.5))); // Fine-tune control left

    Driver.povUpRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5).withVelocityY(-.5))); // Fine-tune control diagonally up and right
    Driver.povUpLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(.5).withVelocityY(.5))); // Fine-tune control diagonally up and left
    Driver.povDownLeft().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5).withVelocityY(.5))); // Fine-tune control diagonally down and left
    Driver.povDownRight().whileTrue(drivetrain.applyRequest(() -> driveFieldCentric.withVelocityX(-.5).withVelocityY(-.5))); // Fine-tune control diagonally down and right

    Driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative).ignoringDisable(true));

    // -- Intake
    Driver.rightTrigger().onTrue(Command_IntakeNoteSequence(false));
    Driver.rightTrigger().onFalse(
        Commands.sequence(
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> Intake.RequestCancelIntake()))
    );
    Driver.rightTrigger().onFalse(Commands.sequence(
        //            Command_AutoPose(),
        Commands.waitSeconds(1),
        Arm.Command_GoToPose(ArmSubsystem.EPose.Stowed)
    ));

    // -- Outtake speaker
    Driver.leftTrigger().onTrue(
        Command_ScoreSpeaker()
    );

    // -- Outtake Amp
    Driver.leftBumper().onTrue(
        Command_ScoreAmp()
    );

    Driver.x().onTrue(Command_DriveForward(-1, .45));

    // -- Align
    Driver.rightBumper().whileTrue(new AutoTagCommand());

    //        Driver.rightBumper().onFalse(Commands.sequence(
    //            Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.amp)
    //        ));

    //        Driver.rightBumper().onFalse(Commands.sequence(
    //            Commands.waitSeconds(1),
    //            Command_AutoPose()
    //        ));

    // -- Testing for autoPosing and Outtaking depending on apriltag
    //        Driver.a().onTrue(Command_AutoPose());
    Driver.b().whileTrue(drivetrain.applyRequest(this::GetAlignToTagRequest));

  }

  // --------------------------------------------------------------------------------------------
  // -- Operator
  // --------------------------------------------------------------------------------------------
  private void ConfigureOperatorBindings()
  {
    Operator.start().onTrue(Arm.Command_ZeroArmEncoder().alongWith(Arm.Command_ZeroPivotEncoder().ignoringDisable(true)));
    Operator.back().whileTrue(Arm.Command_ManualArmControl());



    Operator.a().onTrue(Arm.Command_GoToPose(ArmSubsystem.EPose.Stowed));
    Operator.b().onTrue(Arm.Command_UnstickPivot());
    Operator.x().onTrue(Arm.Command_GoToPose(ArmSubsystem.EPose.Amp));
    Operator.y().onTrue(Arm.Command_GoToPose(ArmSubsystem.EPose.Speaker));

    Operator.povUp().onTrue(Arm.Command_GoToPose(ArmSubsystem.EPose.PreClimb));
    Operator.povDown().onTrue(Arm.Command_Climb());

    Operator.povLeft().onTrue(Arm.Command_SetNeutralMode(NeutralModeValue.Brake).alongWith(Arm.Command_SetNeutralMode(NeutralModeValue.Brake)));
    Operator.povRight().onTrue(Arm.Command_SetNeutralMode(NeutralModeValue.Coast).alongWith(Arm.Command_SetNeutralMode(NeutralModeValue.Coast)));

    Operator.rightBumper().whileTrue(Intake.Command_MoveNote(false));
    Operator.leftBumper().whileTrue(Intake.Command_MoveNote(true));

    Operator.leftTrigger().onTrue(Intake.Command_FeederTakeNote(false));

    Operator.rightTrigger().onTrue(Command_IntakeNoteSequence(true));
    Operator.rightTrigger().onFalse(Arm.Command_GoToPose(ArmSubsystem.EPose.Stowed).andThen(Commands.runOnce(() -> Intake.RequestCancelIntake())));
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

  private void SetDefaultCommands() { drivetrain.setDefaultCommand(drivetrain.applyRequest(this::GetDefaultDriveRequest).ignoringDisable(Constants.FeatureFlags.IgnoringDisabled)); }

  public SwerveRequest GetDefaultDriveRequest()
  {
    var output = GetVelocityForThrottle();

    if (RotationModeIsRobotCentric)
    {
      return driveRobotCentric
          .withVelocityX(output.getX())
          .withVelocityY(output.getY())
          .withRotationalRate(GetDefaultDriveRotationRate());
    }
    else
    {
      return driveFieldCentric
          .withVelocityX(output.getX())
          .withVelocityY(output.getY())
          .withRotationalRate(GetDefaultDriveRotationRate());
    }
  }

  public double GetDefaultDriveRotationRate()
  {
    double magicScalar = 0.02;

    double finalAngleVelocity = -TurnLUT.GetValue(Driver.getRightX()) * MaxRotationsPerSecond * 360;

    return finalAngleVelocity * magicScalar;
  }

  public Pose2d GetVelocityForThrottle()
  {
    double y = Driver.getLeftX();
    double x = Driver.getLeftY();

    double deflection = Math.sqrt(x*x + y*y);
    double deflectionLUT = ThrottleLUT.GetValue(deflection);

    if (deflectionLUT == 0) { return new Pose2d(); }

    double finalX = x * deflectionLUT * SwerveConstants.kSpeedAt12VoltsMps;
    double finalY = y * deflectionLUT * SwerveConstants.kSpeedAt12VoltsMps;

    return new Pose2d(-finalX, -finalY, new Rotation2d());
  }

  private SwerveRequest GetAlignToTagRequest()
  {
    var target = vision.GetBestTarget();
    if (target == null) { return GetDefaultDriveRequest(); }

    Pose3d pose = target.getRobotPose_TargetSpace();
    double posY = pose.getTranslation().getZ();
    double posX = pose.getTranslation().getX();
    double angleY = pose.getRotation().getY();

    double rotationalOffset = 0.05;

    Logger.recordOutput("Target.posX", posX);
    Logger.recordOutput("Target.posY", posY);
    Logger.recordOutput("Target.angleY", angleY);

    return driveRobotCentric
        .withVelocityX(-(posY + 1))
        .withVelocityY(posX * 2)
        .withRotationalRate((angleY + rotationalOffset) * 7);
  }

  public Command Command_ScoreSpeaker()
  {
    return Commands.sequence(
        Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.Speaker),
        Arm.Command_GoToPose(ArmSubsystem.EPose.Stowed)
    );
  }

  public Command Command_ScoreAmp()
  {
    return Commands.sequence(
        Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.Amp),
        Arm.Command_GoToPose(ArmSubsystem.EPose.Stowed)
    );
  }

  private void ConfigureAutoCommands()
  {
    NamedCommands.registerCommand("Arm Score", Arm.Command_GoToPose(ArmSubsystem.EPose.Speaker));
    NamedCommands.registerCommand("Arm Stow", Arm.Command_GoToPose(ArmSubsystem.EPose.Stowed));

    NamedCommands.registerCommand("Intake Note", Command_IntakeNoteSequence(false).withTimeout(2.5));
    NamedCommands.registerCommand("Bump Note", Intake.Command_MoveNote(false).withTimeout(1));
    NamedCommands.registerCommand("Stop Intake Motors", Intake.Command_StopIntake());
    NamedCommands.registerCommand("Condition Stop Intake", Command_ConditionalStowAuto());
    NamedCommands.registerCommand("Shoot Speaker", Intake.Command_Outtake(IntakeSubsystem.EOuttakeType.Speaker).withTimeout(0.5).andThen(Intake.Command_StopIntake()));

  }

  public Command Command_DriveForward(double direction, double time) { return Commands.run(() -> driveFieldCentric.withVelocityY(direction)).withTimeout(time); }

  public Command Command_Stop()
  {
    return Commands.run(() -> driveRobotCentric
        .withVelocityX(0)
        .withVelocityX(0)
        .withRotationalRate(0));
  }

  Command Command_IntakeNoteSequence(boolean fromSource)
  {
    return Commands.sequence(
        Arm.Command_SetPosition(fromSource ? ArmSubsystem.EArmPosition.Source : ArmSubsystem.EArmPosition.Stowed),
        Command_IntakeNote(fromSource)
    );
  }

  public Command Command_ConditionalStowAuto()
  {
    return Commands.runOnce(() ->
                            {
                              System.out.println("Auto Stow: " + Intake.hasGottenNote);
                              if (!Intake.hasGottenNote)
                              {
                                Intake.StopMotors();
                                Arm.PivotMotor.setControl(Arm.PivotRequest.withPosition(ArmSubsystem.EPivotPosition.Stowed.Rotations));
                              }
                            });
  }

  Command Command_AutoOuttake() { return Commands.defer(() -> Intake.Command_Outtake(GetOuttakeType()), Set.of(Intake)); }

  public IntakeSubsystem.EOuttakeType GetOuttakeType()
  {
    var target = Vision.GetBestTarget();
    if (target == null)
    {
      System.out.println("No target");
      return IntakeSubsystem.EOuttakeType.None;
    }
    var id = target.fiducialID;

    if (id == 5 || id == 6)
    {
      System.out.println("Amp Outtake");
      return IntakeSubsystem.EOuttakeType.Amp;
    }
    else if (id == 4 || id == 7)
    {
      System.out.println("Speaker Outtake");
      return IntakeSubsystem.EOuttakeType.Speaker;
    }

    System.out.println("No outtake type");
    return IntakeSubsystem.EOuttakeType.None;

  }

  public Command Command_IntakeNote(boolean fromSource)
  {
    if ( Intake.RightSwitch.get() && Intake.LeftSwitch.get())
    {
      return Commands.sequence(
          Commands.print("Intake Note starting"),

          Commands.runOnce(() ->
                           {
                             Intake.isFeedingNote = false;
                             Intake.IntakeMotor.setControl(Intake.IntakeRequest.withOutput(fromSource ? IntakeSubsystem.EFeedType.IntakeFromSource.DutyCycle : IntakeSubsystem.EFeedType.IntakeFromGround.DutyCycle));
                           }),

          Arm.Command_SetPivotPosition(fromSource ? ArmSubsystem.EPivotPosition.Source : ArmSubsystem.EPivotPosition.Intake),

          Commands.waitSeconds(0.25),

          Commands.print(fromSource ? "At source, looking for note" : "On ground, looking for note"),

          Commands.runOnce(() ->
                           {
                             Intake.currentSpikeCount = 0;
                             Intake.lastCurrent = Intake.IntakeMotor.getStatorCurrent().getValue();
                             if (fromSource)
                             {
                               Limelight.setLEDMode_ForceBlink(VisionConstants.limelight);
                             }
                           }),

          Commands.waitUntil(() ->
                             {
                               double curCurrent = Intake.IntakeMotor.getStatorCurrent().getValue();
                               Logger.recordOutput("Intake.currentDelta", curCurrent);

                               if (curCurrent - Intake.lastCurrent > 15) { Intake.currentSpikeCount++; }

                               Intake.lastCurrent = curCurrent;
                               if (Intake.currentSpikeCount >= 1)
                               {
                                 Intake.isFeedingNote = true;
                                 Intake.hasGottenNote = true;
                                 Limelight.setLEDMode_ForceOff(VisionConstants.limelight);

                                 return true;
                               }

                               return false;
                             }),

          Command_RumbleControllers(),

          Commands.print("Note got - stowing"),

          Commands.runOnce(() -> Arm.PivotMotor.setControl(Arm.PivotRequest.withPosition(ArmSubsystem.EPivotPosition.Stowed.Rotations))),

          Commands.print("Slowing down intake, spinning up feeder"),

          Commands.runOnce(() -> Intake.IntakeMotor.setControl(Intake.IntakeRequest.withOutput(IntakeSubsystem.EFeedType.IntakeToFeeder.DutyCycle))),
          Commands.runOnce(() -> Intake.FeederMotor.set(IntakeSubsystem.EFeedType.FeederTakeNote.DutyCycle)),

          Commands.waitSeconds(0.1).unless(() -> fromSource),

          Intake.Command_FeederTakeNote(true)
          )
          .finallyDo(() ->
                     {
                       if (!Intake.isFeedingNote)
                       {
                         Intake.StopMotors();
                         Limelight.setLEDMode_ForceOff(VisionConstants.limelight);
                       }
                     });
    }

    return Commands.none();
  }

  public void RequestCancelIntake()
  {
    if (Intake.isFeedingNote) { return; }

    CommandScheduler.getInstance().schedule(
        Commands.sequence(
            Intake.Command_StopIntake(),
            Arm.Command_SetPivotPosition(ArmSubsystem.EPivotPosition.Stowed)
        )
    );
  }

}
