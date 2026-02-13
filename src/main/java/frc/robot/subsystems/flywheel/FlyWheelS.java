package frc.robot.subsystems.flywheel;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface FlyWheelS extends Subsystem {
    public Command setVelocity(Supplier<AngularVelocity> speed);
    public Command setVoltage(Supplier<Voltage> volts);
    public Command resetEncoder();

    public AngularVelocity getVelocity();
    public Optional<AngularVelocity> getSetpoint();
    public boolean atSetpoint();
    public Current getCurrent();
}
