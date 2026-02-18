package frc.robot.subsystems.intakeroller;

import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IntakeRollerS extends SubsystemBase {
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command setVoltage(Voltage voltage);
    public abstract Command resetEncoder();
    @Logged
    public abstract Current getCurrent();
}
