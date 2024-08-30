package friarLib3.utility;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

public class PhoenixUtil
{
    // Spams fetching StatusCode until it works or runs out of attempts
    public static boolean spamGetStatusCode(Supplier<StatusCode> function, int numTries)
    {
        StatusCode code = function.get();
        int tries = 0;
        while (code != StatusCode.OK && tries < numTries)
        {
            DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
            code = function.get();
            tries++;
        }
        if (code != StatusCode.OK)
        {
            DriverStation.reportError("Failed to execute Phoenix Pro API call after " + numTries + " attempts", false);
            return false;
        }
        return true;
    }

    public static boolean spamGetStatusCode(Supplier<StatusCode> function)
    {
        return spamGetStatusCode(function, 5);
    }

    public static boolean readAndVerifyConfig(TalonFX talon, TalonFXConfiguration config)
    {
        TalonFXConfiguration readConfig = new TalonFXConfiguration();
        if (!spamGetStatusCode(() -> talon.getConfigurator().refresh(readConfig)))
        {
            DriverStation.reportWarning("Failed to read config for talon [" + talon.getDescription() + "]", false);
            return false;
        }
        else if (!TalonConfigEquality.isEqual(config, readConfig))
        {
            DriverStation.reportWarning("Configuration verification failed for talon [" + talon.getDescription() + "]", false);
            return false;
        } else {
            // configs read and match, Talon OK
            return true;
        }
    }

    // The main function you should use for most purposes
    public static boolean applyMotorConfigs(
        TalonFX motor, TalonFXConfiguration motorConfig, int numTries) {
        for (int i = 0; i < numTries; i++) {
            if (spamGetStatusCode(() -> motor.getConfigurator().apply(motorConfig))) {
                // API says we applied config, lets make sure it's right
                if (readAndVerifyConfig(motor, motorConfig)) {
                    return true;
                } else {
                    DriverStation.reportWarning(
                        "Failed to verify config for talon ["
                            + motor.getDescription()
                            + "] (attempt "
                            + (i + 1)
                            + " of "
                            + numTries
                            + ")",
                        false);
                }
            } else {
                DriverStation.reportWarning(
                    "Failed to apply config for talon ["
                        + motor.getDescription()
                        + "] (attempt "
                        + (i + 1)
                        + " of "
                        + numTries
                        + ")",
                    false);
            }
        }
        DriverStation.reportError(
            "Failed to apply config for talon after " + numTries + " attempts", false);
        return false;
    }
}
