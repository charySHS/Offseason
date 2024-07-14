package friarLib2.Logging;

import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusSignal;

import org.littletonrobotics.junction.Logger;

public enum FriarLogger {

    INSTANCE;

    private Kattio io;
    private int LinesPrinted = 0;

    private FriarLogger()
    {
        this.io = new Kattio(System.in, System.out);
    }

    public static FriarLogger getInstance()
    {
        return INSTANCE;
    }

    public <T>StatusSignal<T> moniterStatusSignal
        (
            StatusSignal<T> signal,
            int CANdeviceID,
            String CANdeviceBus,
            String ItemName,
            String MethodName
        )
    {
        Logger.recordOutput
                  (
                      ItemName + " | " + CANdeviceID + " | " + CANdeviceBus,
                      "Signal in "
                      + signal.getName()
                      + "( "
                      + MethodName
                      + " ): "
                      + signal.getStatus().toString()
                      + " CANdeviceID: "
                      + CANdeviceID
                      + " CANdeviceBus: "
                      + CANdeviceBus
                      + " Device Name: "
                      + signal.getName()
                  );
        if (signal.getStatus().isError())
        {
            println("---- ERROR ----");
            println(
                "Error in "
                + signal.getName()
                + "( "
                + MethodName
                + " )"
                + ": "
                + signal.getStatus().toString()
            );
            println("CANdeviceID: " + CANdeviceID + " CANdeviceBus: " + CANdeviceBus);
            println("Device Name: " + signal.getName());
            println("---- ERROR ----");
        }
        return signal;
    }

    private void println(String message)
    {
        // io.println(message);
        // linesPrinted += 1;
        // if (linesPrinted >= Constants.kLogLinesBeforeFlush) {
        // io.flush();
        // linesPrinted = 0;
        // }
    }

    public void info(String message)
    {
        println("[INFO]: " + message);
    }

    public void warn(String message)
    {
        println("[Warning]: " + message);
        DriverStation.reportWarning("[WARNING]: " + message, false);
    }

    public void error(String message)
    {
        println("[ERROR]: " + message);
        DriverStation.reportError("[ERROR]: " + message, false);
    }

    public void error(String message, ErrorCode errorCode)
    {
        println("[ERROR]: " + message + " | Error Code: " + errorCode);
        DriverStation.reportError("[ERROR]: " + message, false);
        println("Traceback (most recent call last)");
        for (StackTraceElement ste : Thread.currentThread().getStackTrace())
        {
            println(ste + "\n");
        }
    }

}
