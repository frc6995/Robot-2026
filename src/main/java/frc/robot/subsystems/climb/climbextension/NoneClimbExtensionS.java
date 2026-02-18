package frc.robot.subsystems.climb.climbextension;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneClimbExtensionS extends ClimbExtensionS {

    @Override
    public Command setHeight(Supplier<Distance> height) {
        return Commands.none();
    }

    @Override
    public Command setHeightAndStop(Supplier<Distance> height) {
        return Commands.none();
    }

    @Override
    public void setHeightSetpoint(Supplier<Distance> height) {
        return;
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

}
