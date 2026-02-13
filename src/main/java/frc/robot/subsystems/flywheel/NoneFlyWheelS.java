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

public class NoneFlyWheelS implements FlyWheelS {

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
        return DegreesPerSecond.of(-6995);
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
        return Amps.of(-6995);
    }
    
}
