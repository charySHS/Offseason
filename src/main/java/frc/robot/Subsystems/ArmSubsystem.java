package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import friarLib3.utility.DisableSubsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class ArmSubsystem extends DisableSubsystem
{

    LoggableInputs AutoLog;

    static double manualArmControlTarget = Constants.Arm.ManualArmControlTarget;
    static double lowerLimit = Constants.Arm.LowerLimit;
    static double upperLimit = Constants.Arm.UpperLimit;
    static double armTolerance = Constants.Arm.ArmTolerance;

    static double pivotLimitReverse = Constants.Arm.PivotLimitReverse;
    static double pivotLimitBuffer = Constants.Arm.PivotLimitReverseBuffer;
    static double pivotLimitForward = Constants.Arm.PivotLimitForward;
    static double pivotTolerance = Constants.Arm.PivotTolerance;

    public enum EArmPosition
    {
        Amp(0.169),
        Source(-0.05),
        Stowed(lowerLimit),
        ShootSpeaker(-0.058), // TODO: OG was -0.078; Fix why it changed
        //ShootPodium(Constants.Arm.LowerLimit),
        //ShootWing(Constants.Arm.Lowerlimit),
        ClimbFirstPos(upperLimit),
        Trap(lowerLimit);

        public final double Rotations;

        EArmPosition(double rotations) { Rotations = rotations; }
    }

    public enum EPivotPosition
    {
        Amp(0.075),
        Source(-0.237),
        Stowed(pivotLimitReverse + pivotLimitBuffer),
        ShootSpeaker(pivotLimitReverse),
        Climb(-0.25),
        Trap(pivotLimitReverse + pivotLimitBuffer),
        Unstick(-0.166);

        public final double Rotations;
        EPivotPosition(double rotations) { Rotations = rotations; }
    }

    // -- Motors

    private TalonFX LeftMotor;
    private TalonFX RightMotor;
    private TalonFX PivotMotor;

    private final MotionMagicExpoTorqueCurrentFOC PoseRequest =
        new MotionMagicExpoTorqueCurrentFOC(lowerLimit)
            .withSlot(0);
    private final PositionTorqueCurrentFOC ClimbRequest =
        new PositionTorqueCurrentFOC(lowerLimit)
            .withSlot(1);
    private final MotionMagicExpoTorqueCurrentFOC PivotRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    public ArmSubsystem(boolean enabled)
    {
        super(enabled);

        LeftMotor = CreateMotor(Constants.CANivoreBusIDs.ArmLeft.GetID());
        RightMotor = CreateMotor(Constants.CANivoreBusIDs.ArmRight.GetID());
        CreatePivot();

        RightMotor.setControl(new Follower(Constants.CANivoreBusIDs.ArmLeft.GetID(), true));

        LeftMotor.setControl(PoseRequest.withPosition(EArmPosition.Stowed.Rotations));
    }

    @Override
    public void periodic()
    {
        super.periodic();
        Logger.processInputs(this.getClass().getSimpleName(), AutoLog);
        Logger.recordOutput("Arm.LeftPosition", LeftMotor.getPosition().getValue());
        Logger.recordOutput("Arm.RightPosition", RightMotor.getPosition().getValue());
    }

    public double GetArmPosition() { return LeftMotor.getPosition().getValue(); }

    private TalonFX CreateMotor(int deviceID)
    {
        var motor = new TalonFX(deviceID, Constants.CANivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(300)
                .withKI(0)
                .withKD(65)
                .withKS(0)
                .withKA(0)
                .withKV(0)
                .withKG(8));

        configs.withSlot1(
            new Slot1Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(2000)
                .withKI(0) //TODO: add some KI to make sure we fully climb (probably about 150ish)
                .withKD(65)
                .withKS(8)
                .withKA(0)
                .withKV(0)
                .withKG(27));

        configs.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(5)
                .withMotionMagicExpo_kA(3)
                .withMotionMagicExpo_kV(5)
                .withMotionMagicJerk(1000));

        configs.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(117.6));

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(upperLimit)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(lowerLimit));

        motor.getConfigurator().apply(configs);

        motor.setPosition(lowerLimit);
        return motor;

    }

    private void CreatePivot()
    {
        PivotMotor = new TalonFX(Constants.CANivoreBusIDs.IntakePivot.GetID(), Constants.CANivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(1500)
                .withKI(0)
                .withKD(200)
                .withKS(10)
                .withKA(0)
                .withKV(0)
                .withKG(22));

        configs.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(4)
                .withMotionMagicExpo_kA(5)
                .withMotionMagicExpo_kV(5)
                .withMotionMagicJerk(1000));

        configs.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(60));


        configs.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(pivotLimitForward)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(pivotLimitReverse));

        PivotMotor.getConfigurator().apply(configs);
        PivotMotor.setPosition(pivotLimitReverse);
    }

    // ----------------------------------------------------------------------------------------
    // -- Commands
    // ----------------------------------------------------------------------------------------

    public Command Command_SetPosition(EArmPosition position) { return Command_GoToPosition(position.Rotations); }

    public Command Command_GoToPosition(double armPosition)
    {
        var pos = MathUtil.clamp(armPosition, lowerLimit, upperLimit);
        return
            run(() ->
                {
                    Logger.recordOutput("Arm.Target", pos);
                    LeftMotor.setControl(PoseRequest.withPosition(pos));
                })
                .until(() ->
                       {
                           double actualRotation = LeftMotor.getPosition().getValue();
                           Logger.recordOutput("Arm.Error", pos - actualRotation);
                           return MathUtil.isNear(pos, actualRotation, armTolerance);
                       });
    }

    public double GetPivotPos() { return PivotMotor.getPosition().getValue(); }

    public Command Command_SetPivotPosition(EPivotPosition position) { return Command_GoToPivotPosition(position.Rotations); }

    public Command Command_GoToPivotPosition(double position)
    {
        var pos = MathUtil.clamp(position, pivotLimitReverse, pivotLimitForward);

        return run(() ->
                   {
                       Logger.recordOutput("Pivot.Target", pos);
                       PivotMotor.setControl(PivotRequest.withPosition(pos));
                   })
            .until(() ->
                   {
                       double actualRotation = PivotMotor.getPosition().getValue();
                       return MathUtil.isNear(pos, actualRotation, pivotTolerance);
                   });
    }

    public Command Command_SetNeutralMode(NeutralModeValue mode) { return runOnce(() -> PivotMotor.setNeutralMode(mode)).ignoringDisable(Constants.FeatureFlags.IgnoringDisabled); }

    public Command Command_UnstickPivot()
    {
        return Commands.sequence(
            Command_SetPivotPosition(EPivotPosition.Unstick),
            Command_SetPivotPosition(EPivotPosition.Stowed));
    }

    public Command Command_ZeroPivotEncoder() { return runOnce(() -> PivotMotor.setPosition(pivotLimitReverse)).ignoringDisable(Constants.FeatureFlags.IgnoringDisabled); }

    public Command Command_ZeroArmEncoder() { return runOnce(() -> LeftMotor.setPosition(lowerLimit)).ignoringDisable(Constants.FeatureFlags.IgnoringDisabled); }

    public Command Command_ManualArmControl()
    {
        return runOnce(() -> Constants.Arm.ManualArmControlTarget = LeftMotor.getPosition().getValue())
            .andThen(run(() ->
        {
            double y = RobotContainer.Operator.getLeftY() * 0.001;
            if (Math.abs(y) < 0.001) { return; }

            Constants.Arm.ManualArmControlTarget = MathUtil.clamp(Constants.Arm.ManualArmControlTarget + y, lowerLimit, upperLimit);
            LeftMotor.setControl(ClimbRequest.withPosition(Constants.Arm.ManualArmControlTarget));
        }));
    }

}
