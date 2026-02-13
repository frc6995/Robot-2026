package frc.robot.subsystems.intakepivot;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakePivotS extends Subsystem {
    public Command setAngle(Supplier<Angle> angle);
    public Command setAngle(Angle angle);
    public Command setVoltage(Supplier<Voltage> voltage);
    public Command sysId();
    public Command resetEncoder();

    public boolean isIntakeDeployed();

    public Angle getAngle();
}
