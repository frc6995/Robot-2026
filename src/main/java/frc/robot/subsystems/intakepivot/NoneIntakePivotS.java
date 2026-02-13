package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneIntakePivotS extends IntakePivotS {

    @Override
    public Command setAngle(Supplier<Angle> angle) {
        return Commands.none();
    }

    @Override
    public Command setAngle(Angle angle) {
        return Commands.none();
    }

    @Override
    public Command setVoltage(Supplier<Voltage> voltage) {
        return Commands.none();
    }

    @Override
    public Command sysId() {
        return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public Angle getAngle() {
        return Degrees.of(-6995);
    }

    @Override
    public boolean isIntakeDeployed() {
        return false;
    }
    
}
