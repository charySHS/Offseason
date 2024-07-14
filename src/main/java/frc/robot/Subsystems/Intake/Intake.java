package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.*;

public class Intake extends SubsystemBase {

    public enum EOuttakeType
    {
        amp(-0.5),
        speaker(-1), // known value = -60, to testing value: 74
        None(0),
        trap(0);

        private final double DutyCycle;
        EOuttakeType(double dutyCycle) { DutyCycle = dutyCycle; }
    }

    enum EFeedType
    {
        IntakeFromGround(0.6),
        IntakeFromSource(0.4),
        SendToFeeder(0.3),
        FeederTake(-0.075),
        FeederReturn(0.5);

        private final double DutyCycle;
        EFeedType(double dutyCycle) { DutyCycle = dutyCycle; }
    }

    // -- Motors

}
