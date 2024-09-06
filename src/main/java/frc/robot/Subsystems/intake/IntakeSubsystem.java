package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import friarLib3.utility.DisableSubsystem;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakeSubsystem extends DisableSubsystem
{
    private IntakeIO intakeIO;
    private IntakeIO.IntakeIOInputs intakeIOAutoLogged = new IntakeIO.IntakeIOInputs();
    private SysIdRoutine intakeSysIDRoutine;

    LoggableInputs AutoLog;

    public enum EOuttakeType
    {
        Amp(-0.5),
        Speaker(-1),
        None(0),
        Trap(0);

        public final double DutyCycle;

        EOuttakeType(double dutyCycle)
        {
            DutyCycle = dutyCycle;
        }
    }

    public enum EFeedType
    {
        IntakeFromGround(0.6),
        IntakeFromSource(0.4),
        IntakeToFeeder(0.3),
        FeederTakeNote(-0.075),
        FeederGiveNote(0.5);

        public final double DutyCycle;

        EFeedType(double dutyCycle)
        {
            DutyCycle = dutyCycle;
        }
    }

    // -- Motors
    public TalonFX IntakeMotor;
    public CANSparkFlex FeederMotor;
    public SparkPIDController FeederMotorPID;

    // -- Phoenix Requests
    public final DutyCycleOut IntakeRequest = new DutyCycleOut(0);

    public final DigitalInput LeftSwitch = new DigitalInput(1);
    public final DigitalInput RightSwitch = new DigitalInput(2);


    public double lastCurrent = 0;
    public int currentSpikeCount = 0;

    public boolean isFeedingNote = false;
    public boolean hasGottenNote = false;

    public boolean GetIsFeedingNote()
    {
        return isFeedingNote;
    }

    public IntakeSubsystem(boolean enabled, IntakeIO intakeIO)
    {
        super(enabled);

        this.intakeIO = intakeIO;

        intakeSysIDRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(0.2)
                         .per(Seconds.of(1)),
                    Volts.of(6),
                    null,
                    (state) -> SignalLogger.writeString("State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    (volts) ->
                        intakeIO
                            .GetIntakeMotor()
                            .setControl(IntakeRequest.withOutput(volts.in(Volts))),
                    null,
                    this
                )
            );

        CreateIntakeMotor();
        CreateFeederMotor();
    }

    @Override
    public void periodic()
    {
        super.periodic();
        intakeIO.UpdateInputs(intakeIOAutoLogged);
        Logger.processInputs(this.getClass()
                                 .getSimpleName(), (LoggableInputs) intakeIOAutoLogged);
    }

    public Command SetVoltage(double voltage, double feederVoltage)
    {
        return this.run(
                       () ->
                       {
                           intakeIO.SetIntakeVoltage(voltage);
                           intakeIO.SetFeederVoltage(feederVoltage);
                       }
                   )
                   .finallyDo(intakeIO::off);
    }

    public Command SetVelocity(double velocity, double feederVelocity)
    {
        return this.run(
                       () ->
                       {
                           intakeIO.SetIntakeVelocity(velocity);
                           intakeIO.SetFeederVelocity(feederVelocity);
                       }
                   )
                   .finallyDo(intakeIO::off);
    }

    public Command SetIntakeVoltage(double voltage)
    {
        return this.run(() -> intakeIO.SetIntakeVoltage(voltage))
                   .finallyDo(intakeIO::off);
    }

    public Command SetIntakeVelocity(double velocity)
    {
        return this.run(() -> intakeIO.SetIntakeVelocity(velocity))
                   .finallyDo(intakeIO::off);
    }

    public Command SetFeederVoltage(double voltage)
    {
        return this.run(() -> intakeIO.SetFeederVoltage(voltage))
                   .finallyDo(intakeIO::off);
    }

    public Command SetFeederVelocity(double velocity)
    {
        return this.run(() -> intakeIO.SetFeederVelocity(velocity))
                   .finallyDo(intakeIO::off);
    }

    public Command off() { return this.runOnce(intakeIO::off); }


    //  ----------------------------------------------------------------------------------------------
    // -- Intake Motor
    // -----------------------------------------------------------------------------------------------

    private void CreateIntakeMotor()
    {
        IntakeMotor = new TalonFX(Constants.CANivoreBusIDs.IntakeMotor.GetID(), Constants.CANivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withMotorOutput
                   (
                       new MotorOutputConfigs()
                           .withNeutralMode(NeutralModeValue.Brake));

        configs.withSlot0
                   (
                       new Slot0Configs()
                           .withGravityType(GravityTypeValue.Elevator_Static)
                           .withKP(10)
                           .withKI(0)
                           .withKD(0)
                           .withKS(40)
                           .withKA(0)
                           .withKV(0.25)
                           .withKG(0)
                   );

        IntakeMotor.getConfigurator().apply(configs);

        IntakeMotor.stopMotor();
    }

    // --------------------------------------------------------------------------------------------
    // -- Feeder Motor
    // --------------------------------------------------------------------------------------------
    private void CreateFeederMotor()
    {
        FeederMotor = new CANSparkFlex(2, CANSparkLowLevel.MotorType.kBrushless);

        FeederMotor.restoreFactoryDefaults();
        FeederMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        FeederMotor.setInverted(true);

        FeederMotorPID = FeederMotor.getPIDController();

        FeederMotorPID.setP(0.05);
        FeederMotorPID.setI(0.0000001);
        FeederMotorPID.setD(0.01357);
        FeederMotorPID.setIZone(0);
        FeederMotorPID.setFF(0.000015);
        FeederMotorPID.setOutputRange(-1, 1);

        FeederMotor.burnFlash();

        FeederMotor.stopMotor();
    }

    public void RequestCancelIntake() {}

    public void StopMotors()
    {
        IntakeMotor.stopMotor();
        FeederMotor.stopMotor();
    }

    public Command Command_Outtake(EOuttakeType outtakeType)
    {
        return Commands.sequence(
            runOnce(() -> IntakeMotor.setControl(IntakeRequest.withOutput(outtakeType.DutyCycle))),
            Commands.waitSeconds(0.25),
            runOnce(() -> FeederMotor.set(0.5)),
            Commands.waitSeconds(0.5),
            Command_StopIntake()
        );
    }

    public Command Command_StopIntake() { return runOnce(this::StopMotors); }

    public Command Command_MoveNote(boolean forward)
    {
        return startEnd(
            () -> {
                IntakeMotor.setControl(IntakeRequest.withOutput(forward ? -0.5: 0.5));
                FeederMotor.set(forward ? 0.3: -0.3);
            },
            this::StopMotors
        );
    }

    public Command Command_FeederTakeNote(boolean skipWaitForSpinUp)
    {
        return Commands.sequence(
            runOnce(() -> FeederMotor.set(EFeedType.FeederTakeNote.DutyCycle)),

            Commands.waitSeconds(0.25).unless(() -> skipWaitForSpinUp),

            runOnce(() -> {
                currentSpikeCount = 0;
                lastCurrent = FeederMotor.getOutputCurrent();
                IntakeMotor.setControl(IntakeRequest.withOutput(EFeedType.IntakeToFeeder.DutyCycle));
            }),

            Commands.waitUntil(() -> {
                        double curCurrent = FeederMotor.getOutputCurrent();
                        Logger.recordOutput("Intake.CurrentDelta", curCurrent - lastCurrent);

                        if (curCurrent - lastCurrent > 5)
                        {
                            currentSpikeCount++;
                        }
                        lastCurrent = curCurrent;

                        return currentSpikeCount >= 1;
                    })
                    .withTimeout(1),

            Commands.waitSeconds(0.25)

        ).finallyDo(() -> {
            StopMotors();
            isFeedingNote = false;
        });
    }

}
