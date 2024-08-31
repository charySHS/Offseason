package friarLib3.general;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class XboxStalker
{
    public static void stalk(CommandXboxController driver, CommandXboxController operator)
    {
        Logger.recordOutput("DriverStatus", driver.getHID().getButtonCount() > 0);
        Logger.recordOutput("OperatorStatus", operator.getHID().getButtonCount() > 0);
    }

}
