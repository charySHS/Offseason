package friarLib2.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;

/**
 * Run the specified command until the specified condition returns true or
 * until that command finishes
 */
public class RunUntil extends ParallelRaceGroup {

    public void RunUntilCommand(Command command, BooleanSupplier endCondition)
    {
        addCommands(
            command,
            new WaitUntilCommand(endCondition)
        );
    }
}
