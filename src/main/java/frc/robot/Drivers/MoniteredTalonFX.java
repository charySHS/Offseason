package frc.robot.Drivers;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import friarLib2.Logging.FriarLogger;

public class MoniteredTalonFX extends TalonFX {

    private final int CANdeviceID;
    private final String CANdeviceBus;

    public MoniteredTalonFX(int canDevID, String canDevBus)
    {
        super(canDevID, canDevBus);
        this.
    }
}
