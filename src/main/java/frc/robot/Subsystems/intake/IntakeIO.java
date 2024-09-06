package frc.robot.Subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO
{
    @AutoLog
    public static class IntakeIOInputs
    {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorVelocity = 0.0;
        public double intakeMotorStatorCurrent = 0.0;
        public double intakeMotorSupplyCurrent = 0.0;
        public double intakeMotorTemperature = 0.0;

        public double feederMotorVoltage = 0.0;
        public double feederMotorVelocity = 0.0;
        public double feederMotorStatorCurrent = 0.0;
        public double feederMotorSupplyCurrent = 0.0;
        public double feederMotorTemperature = 0.0;

        public boolean hasCurrentSpiked = false;
    }

    public default TalonFX GetIntakeMotor() { return new TalonFX(0, ""); }

    public default CANSparkFlex GetFeederMotor() { return new CANSparkFlex(0, CANSparkLowLevel.MotorType.kBrushless); }

    public default void UpdateInputs(IntakeIOInputs inputs) {}

    public default void SetIntakeVoltage(double voltage) {}

    public default void SetIntakeVelocity(double velocity) {}

    public default void SetFeederVoltage(double voltage) {}

    public default void SetFeederVelocity(double velocity) {}

    public default void off() {}

    public default boolean hasCurrentSpiked() { return false; }
}
