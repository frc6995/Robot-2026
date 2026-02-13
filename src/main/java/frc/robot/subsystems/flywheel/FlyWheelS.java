package frc.robot.subsystems.flywheel;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class FlyWheelS extends SubsystemBase {
    public abstract Command setVelocity(Supplier<AngularVelocity> speed);
    public abstract Command setVoltage(Supplier<Voltage> volts);
    public abstract Command resetEncoder();

    public abstract AngularVelocity getVelocity();
    public abstract Optional<AngularVelocity> getSetpoint();
    public abstract boolean atSetpoint();
    public abstract Current getCurrent();
}
