package frc.robot.subsystems.turret;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface TurretS extends Subsystem {
    public Command setAngle(Supplier<Rotation2d> angle);
    public Command setAngle(Rotation2d angle);
    public Command setVoltage(Supplier<Voltage> voltage);
    public Command setVoltage(Voltage voltage);
    public Command sysId();
    public Command resetEncoder();
    public Command aimAtFieldPose(Supplier<Translation2d> targetLocation, Supplier<Pose2d> drivebasePose);
    public Command aimAtHub();
    public Command setAngleFieldRelative(Supplier<Rotation2d> targetFieldRelativeAngle,Supplier<Rotation2d> drivebaseAngle);
    public Command driveToHome();

    public Supplier<Rotation2d> applyDynamicLimits(Supplier<Rotation2d> angle);
    public Optional<Angle> getSetpoint();
    public Angle getAngle();
    public boolean atSetpoint();
    public Current getCurrent();

}
