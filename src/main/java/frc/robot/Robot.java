// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.net.PortForwarder;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import monologue.Logged;
import monologue.Monologue;
import com.pathplanner.lib.commands.FollowPathCommand;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.Limelight.Limelight;
import friarLib3.utility.NT4PublisherNoFMS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends LoggedRobot implements Logged {

  private Command AutonomousCommand;
  private RobotContainer RobotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit()
  {
    RobotController.setBrownoutVoltage(5.6); // we ball

    RobotContainer = new RobotContainer();
    if (Constants.EnableMonologue)
    {
      // Init monologue
      Monologue.setupMonologue(this, "MonologueRobot", Constants.MonologueFileOnly, Constants.MonologueLazyLogging);
    } else {
      System.out.println("monologue is disabled - not configuring logging or config");
    }
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    if (isReal())
    {
      System.out.println("Robot is real, forcing mode to REAL");
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser.logs"));
      Logger.addDataReceiver(new NT4PublisherNoFMS());
      new PowerDistribution(
          1, PowerDistribution.ModuleType.kRev
      );
    } else if (isSimulation()) {
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Logging active commands
    Map<String, Integer> CommandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> LogCommandFunction =
        (Command command, Boolean active) -> {
      String name = command.getName();
      int count = CommandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      CommandCounts.put(name, count);

      Logger.recordOutput(
          "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
      Logger.recordOutput("CommandsAll/" + name, count > 0);
      };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              LogCommandFunction.accept(command, true);
            }
        );
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              LogCommandFunction.accept(command, false);
            }
        );
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              LogCommandFunction.accept(command, false);
            }
        );

    if (Constants.EnableAdvKit)
    {
      Logger.start();
    }
    FollowPathCommand.warmupCommand().schedule();

    // Limelight Init
    for (int port = 5800; port <= 5807; port++)
    {
      PortForwarder.add(port, "10.33.9.11", port);
    }

    RobotContainer.drivetrain.getDaqThread().setThreadPriority(99);

    System.out.println("Robot Initialized!");

    CommandScheduler.getInstance().schedule(Commands.sequence(
        Commands.runOnce(() -> Limelight.setLEDMode_ForceBlink("")),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> Limelight.setLEDMode_ForceOff(""))
    ).ignoringDisable(true));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing finished or
    // interrupted commands, and running subsystem periodic() methods. This must be called from the
    // robot's periodic block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (Constants.EnableMonologue) {
      Monologue.setFileOnly(DriverStation.isFMSAttached());
      Monologue.updateAll();
    }

    SmartDashboard.putNumber("Robot Yaw", RobotContainer.drivetrain.getPigeon2().getYaw().getValue());

    RobotContainer.periodic(Robot.defaultPeriodSecs);
  }
}
