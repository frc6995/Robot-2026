package frc.robot.subsystems.agitator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneAgitatorS extends AgitatorS{

    @Override
    public Command setVoltage(Supplier<Voltage> volts) {
        return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(6.995);
    }

    @Override
    public Current getCurrent() {
        return Amps.of(69.95);
    }
    
}
