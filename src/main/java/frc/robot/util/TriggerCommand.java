package frc.robot.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TriggerCommand extends Command {
    private EventLoop m_eventLoop = new EventLoop();

    private ArrayList<Command> m_commands = new ArrayList<Command>();

    private TriggerCommand(Runnable toRun) {
        bind(toRun, () -> true);
    }

    private TriggerCommand(Command toRun) {
        bind(toRun, () -> true);
    }

    public static TriggerCommand create(Runnable toRun) {
        return new TriggerCommand(toRun);
    }

    public static TriggerCommand create(Command toRun) {
        return new TriggerCommand(toRun);
    }

    public TriggerCommand bind(Runnable toRun, BooleanSupplier booleanSupplier) {
        m_eventLoop.bind(
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
        if(!m_commands.contains(command)) {
            for(Subsystem requirement : command.getRequirements()) {
                if(getRequirements().contains(requirement)) 
                    throw new RuntimeException("Multiple Commands in TriggerCommands cannot require the same subsystem");
            }
        }
        m_commands.add(command);
        m_eventLoop.bind(
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

    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requirements = new HashSet<Subsystem>();

        for(Command command : m_commands) {
            requirements.addAll(command.getRequirements());
        }
        return requirements;
    } 
}