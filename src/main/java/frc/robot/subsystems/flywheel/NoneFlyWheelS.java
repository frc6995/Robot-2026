package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneFlyWheelS extends FlyWheelS {

    @Override
    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return Commands.none();
    }

    @Override
    public Command setVoltage(Supplier<Voltage> volts) {
        return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public AngularVelocity getVelocity() {
        return DegreesPerSecond.zero();
    }

    @Override
    public Optional<AngularVelocity> getSetpoint() {
        return Optional.empty();
    }

    @Override
    public boolean atSetpoint() {
        return true;
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
