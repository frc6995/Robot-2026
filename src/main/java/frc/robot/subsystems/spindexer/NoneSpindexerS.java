package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneSpindexerS extends SpindexerS {

    @Override
    public Command setVoltage(Supplier<Voltage> voltage) {
        return Commands.none();
    }

    @Override
    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public Current getCurrent() {
        return Amps.zero();
    }
    
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        defaultCommand.addRequirements(this);
        super.setDefaultCommand(defaultCommand);
    }
}
