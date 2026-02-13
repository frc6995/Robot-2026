package frc.robot.subsystems.indexer;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IndexerS extends Subsystem {
    public Command setVoltage(Supplier<Voltage> voltage);
    public Command resetEncoder();

    public Current getCurrent();
}
