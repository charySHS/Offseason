package friarLib2.utility;

import edu.wpi.first.wpilibj.DriverStation;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.robot.Robot;

// Subclass of NT4Publisher that doesn't publish if FMS is attached

public class NT4PublisherNoFMS extends NT4Publisher {
    public NT4PublisherNoFMS()
    {
        super();
    }

    @Override
    public void putTable(LogTable table)
    {
        if (Robot.isReal() && DriverStation.isFMSAttached())
        {
            return;
        }
        super.putTable(table);
    }
}
