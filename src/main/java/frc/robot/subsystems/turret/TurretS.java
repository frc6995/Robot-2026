package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TurretS extends SubsystemBase {
    public abstract Command setAngle(Supplier<Rotation2d> angle);
    public abstract Command setAngle(Rotation2d angle);
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command setVoltage(Voltage voltage);
    public abstract Command sysId();
    public abstract Command resetEncoder();
    public abstract Command aimAtFieldPose(Supplier<Translation2d> targetLocation, Supplier<Pose2d> drivebasePose);
    @SuppressWarnings("unchecked")
    public abstract Command aimAtClosestPose(Supplier<Pose2d> drivebasePose, Supplier<Translation2d>... translations);
    public abstract Command aimAtHub();
    public abstract Command setAngleFieldRelative(Supplier<Rotation2d> targetFieldRelativeAngle,Supplier<Rotation2d> drivebaseAngle);
    public abstract Command driveToHome();

    public abstract Supplier<Angle> applyDynamicLimits(Supplier<Angle> angle);
    public abstract Optional<Angle> getSetpoint();
    @Logged
    public abstract Angle getAngle();
    @Logged
    public abstract boolean atSetpoint();
    @Logged
    public abstract Current getSupplyCurrent();

}
