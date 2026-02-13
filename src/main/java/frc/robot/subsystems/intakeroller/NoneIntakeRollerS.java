package frc.robot.subsystems.intakeroller;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneIntakeRollerS extends IntakeRollerS {

    @Override
    public Command setVoltage(Supplier<Voltage> voltage) {
        return Commands.none();
    }

    @Override
    public Command setVoltage(Voltage voltage) {
        return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public Current getCurrent() {
        return Amps.of(-6995);
    }
    
}
