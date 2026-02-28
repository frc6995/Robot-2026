package frc.robot.subsystems.climb.climbextension;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class ClimbExtensionS extends SubsystemBase {
    public abstract Command setHeight(Supplier<Distance> height);
    public abstract Command setHeightAndStop(Supplier<Distance> height);
    public abstract void setHeightSetpoint(Supplier<Distance> height);
    public abstract Command resetEncoder();

}
