package frc.robot.subsystems.climb.climbpivot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ClimbPivotS extends SubsystemBase {
    public abstract Command setAngle(Supplier<Angle> angle);
    public abstract Command setVoltage(Supplier<Voltage> volts);
    public abstract Command resetEncoder();
    public abstract Optional<Angle> getSetpoint();
    public abstract boolean isStingerReady();

}
