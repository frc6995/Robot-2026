package frc.robot.subsystems.indexer;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IndexerS extends SubsystemBase {
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command resetEncoder();

    public abstract Current getCurrent();
}
