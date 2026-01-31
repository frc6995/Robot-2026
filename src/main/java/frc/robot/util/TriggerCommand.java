package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TriggerCommand extends Command {
    private EventLoop eventLoop = new EventLoop();

    private TriggerCommand(Runnable toRun) {
        bind(toRun, () -> true);
    }

    private TriggerCommand(Command toRun) {
        bind(toRun, () -> true);
    }

    public static TriggerCommand create(Runnable toRun) {
        return new TriggerCommand(toRun);
    }

    public TriggerCommand bind(Runnable toRun, BooleanSupplier booleanSupplier) {
        eventLoop.bind(
            new Runnable() {
                @Override
                public void run() {
                    if(booleanSupplier.getAsBoolean()) {
                        toRun.run();
                    }
                }
            }
        );
        return this;
    }

    public TriggerCommand bind(Command command, BooleanSupplier booleanSupplier) {
        eventLoop.bind(
            new Runnable() {
                public void run() {
                    if(booleanSupplier.getAsBoolean()) {
                        CommandScheduler.getInstance().schedule(command);
                    }
                }
            }
        );
        return this;
    }
}
