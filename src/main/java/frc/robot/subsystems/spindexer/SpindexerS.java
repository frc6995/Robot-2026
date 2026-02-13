package frc.robot.subsystems.spindexer;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SpindexerS extends Subsystem {
    public Command setVoltage(Supplier<Voltage> voltage);
    public Command setVelocity(Supplier<AngularVelocity> speed);
    public Command resetEncoder();

    public Current getCurrent();
}
