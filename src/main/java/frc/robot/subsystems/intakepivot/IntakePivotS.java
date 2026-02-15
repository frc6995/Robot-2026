package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotVisualizer;

public abstract class IntakePivotS extends SubsystemBase {
    public abstract Command setAngle(Supplier<Angle> angle);
    public abstract Command setAngle(Angle angle);
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command sysId();
    public abstract Command resetEncoder();

    public abstract boolean isIntakeDeployed();

    public abstract Angle getAngle();

    @Override
    public void periodic() {
        // double currentAngleRad = getAngle().in(Radians);
        // RobotVisualizer.updateIntake(currentAngleRad);
    }
}
