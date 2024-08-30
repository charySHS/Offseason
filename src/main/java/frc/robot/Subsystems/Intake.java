package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
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

public class Intake extends DisableSubsystem
{
    LoggableInputs AutoLog;

    public enum EOuttakeType
    {
        Amp(-0.5),
        Speaker(-1),
        None(0),
        Trap(0);

        private final double DutyCycle;
        EOuttakeType(double dutyCycle) { DutyCycle = dutyCycle; }
    }

    enum EFeedType
    {
        IntakeFromGround(0.6),
        IntakeFromSource(0.4),
        IntakeToFeeder(0.3),
        FeederTakeNote(-0.075),
        FeederGiveNote(0.5);

        private final double DutyCycle;
        EFeedType(double dutyCycle) { DutyCycle = dutyCycle; }
    }

    // -- Motors
    private TalonFX IntakeMotor;
    private CANSparkFlex FeederMotor;
    private SparkPIDController FeederMotorPID;

    // -- Phoenix Requests
    private final DutyCycleOut IntakeRequest = new DutyCycleOut(0);

    private final DigitalInput LeftSwitch = new DigitalInput(1);
    private final DigitalInput RightSwitch = new DigitalInput(2);


    double lastCurrent = 0;
    int currentSpikeCount = 0;

    boolean isFeedingNote = false;
    boolean hasGottenNote = false;

    public boolean GetIsFeedingNote() { return isFeedingNote; }

    public Intake(boolean disabled)
    {
        super(disabled);
        CreateIntakeMotor();
        CreateFeederMotor();
    }

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

    @Override
    public void periodic()
    {
        super.periodic();
        Logger.processInputs(this.getClass().getSimpleName(), AutoLog);
    }

    public void RequestCancelIntake() {}

    private void StopMotors()
    {
        IntakeMotor.stopMotor();
        FeederMotor.stopMotor();
    }

}
