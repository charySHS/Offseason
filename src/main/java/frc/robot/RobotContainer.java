// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FeatureFlags;

import friarLib3.math.LookupTable;

public class RobotContainer
{
  private SendableChooser<Command> autoChooser;

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
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
}
