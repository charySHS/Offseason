package friarLib2.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Run the command in the constructor for the given amount of seconds
 */

public class RunForTime extends Command {

    Timer timer;

    Command command;
    double seconds;

    public RunForTime (Command command, double seconds)
    {
        this.command = command;
        this.seconds = seconds;

        timer = new Timer();
    }

    @Override
    public void initialize()
    {
        timer.reset();
        timer.start();
        command.initialize();
    }

    @Override
    public void execute()
    {
        command.execute();
    }

    // Called once command ends or is interrupted
    @Override
    public void end(boolean interrupted)
    {
        command.end(interrupted);
    }

    // Returns true when command should end
    @Override
    public boolean isFinished()
    {
        return timer.get() >= seconds || command.isFinished();
    }
}
