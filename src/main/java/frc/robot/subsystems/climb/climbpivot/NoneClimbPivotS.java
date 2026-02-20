package frc.robot.subsystems.climb.climbpivot;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneClimbPivotS extends ClimbPivotS {

    @Override
    public Command setAngle(Supplier<Angle> angle) {
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

}
