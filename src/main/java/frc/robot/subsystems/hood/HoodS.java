package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterController.ShooterTargetData;

public abstract class HoodS extends SubsystemBase {
    public abstract Command setAngle(Supplier<Angle> angle);
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command sysId();
    public abstract Command autoHoodAngle();
    public abstract Command resetEncoder();

    public abstract Angle applyDynamicLimits(Angle targetAngle, Pose2d pose);
    public abstract Optional<Angle> getSetpoint();
    @Logged
    public abstract boolean isHoodSafe();
    @Logged
    public abstract boolean isHoodReady();

    public abstract Command runSOTF(Supplier<ShooterTargetData> dataSupplier);

}
