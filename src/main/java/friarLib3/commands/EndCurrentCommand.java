package friarLib3.commands;

import edu.wpi.first.wpilibj2.command.Command;

import friarLib3.general.DebugCommandBase;

public class EndCurrentCommand extends DebugCommandBase {

    Command currentCommand;

    public EndCurrentCommand(Command command)
    {
        currentCommand = command;
        currentCommand.end(true);
    }
}
