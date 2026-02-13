package frc.robot.subsystems.hood;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface HoodS extends Subsystem {
    public Command setAngle(Supplier<Angle> angle);
    public Command setVoltage(Supplier<Voltage> voltage);
    public Command sysId();
    public Command autoHoodAngle();
    public Command resetEncoder();

    public Angle applyDynamicLimits(Angle targetAngle, Pose2d pose);
    public Angle getAutoHoodAngle();
    public Optional<Angle> getSetpoint();
    
    public boolean isHoodSafe();
    public boolean isHoodReady();
}
