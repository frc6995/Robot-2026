package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NoneTurretS extends TurretS {

    @Override
    public Command setAngle(Supplier<Rotation2d> angle) {
        return Commands.none();
    }

    @Override
    public Command setAngle(Rotation2d angle) {
        return Commands.none();
    }

    @Override
    public Command setVoltage(Supplier<Voltage> voltage) {
        return Commands.none();
    }

    @Override
    public Command setVoltage(Voltage voltage) {
        return Commands.none();
    }

    @Override
    public Command sysId() {
        return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public Command aimAtFieldPose(Supplier<Translation2d> targetLocation, Supplier<Pose2d> drivebasePose) {
        return Commands.none();
    }

    @SuppressWarnings("unchecked")
    @Override
    public Command aimAtClosestPose(Supplier<Pose2d> drivebasePose, Supplier<Translation2d>... translations) {
        return Commands.none();
    }

    @Override
    public Command aimAtHub() {
        return Commands.none();
    }

    @Override
    public Command setAngleFieldRelative(Supplier<Rotation2d> targetFieldRelativeAngle,
            Supplier<Rotation2d> drivebaseAngle) {
        return Commands.none();
    }

    @Override
    public Command driveToHome() {
        return Commands.none();
    }

    @Override
    public Supplier<Rotation2d> applyDynamicLimits(Supplier<Rotation2d> angle) {
        return () -> Rotation2d.fromDegrees(0);
    }

    @Override
    public Optional<Angle> getSetpoint() {
        return Optional.empty();
    }

    @Override
    public Angle getAngle() {
        return Degrees.of(-6995);
    }

    @Override
    public boolean atSetpoint() {
        return true;
    }

    @Override
    public Current getCurrent() {
        return Amps.of(-6995);
    }
    
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        defaultCommand.addRequirements(this);
        super.setDefaultCommand(defaultCommand);
    }
}
