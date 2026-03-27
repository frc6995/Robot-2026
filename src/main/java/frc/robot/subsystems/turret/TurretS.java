package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.RealTurretS.TurretConstants;
import frc.robot.util.POI;
import frc.robot.util.RobotVisualizer;
import frc.robot.util.TriggerUtil;
import frc.robot.util.ShooterController.ShooterTargetData;
import frc.robot.util.UnitUtil;

public abstract class TurretS extends SubsystemBase {
    protected Supplier<Pose2d> robotPose;
    protected Supplier<ChassisSpeeds> robotSpeeds;
    protected BooleanSupplier isIntakeDeployed;

    protected Rotation2d setpoint = Rotation2d.kZero;

    public TurretS(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds, BooleanSupplier isIntakeDeployed) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.isIntakeDeployed = isIntakeDeployed;
    }

    public abstract Command setAngle(Supplier<Rotation2d> angle);
    public abstract Command setAngle(Rotation2d angle);
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command setVoltage(Voltage voltage);
    public abstract Command sysId();
    public abstract Command resetEncoder();
    public abstract Optional<Angle> getSetpoint();
    @Logged
    public abstract Angle getAngle();
    @Logged
    public abstract Current getSupplyCurrent();

    public Command runSOTF(Supplier<ShooterTargetData> solution) {
        return setAngle(() -> solution.get().turretAngle);
    }

    /**
     * aims the turret at a given field pose
     * 
     * @param targetLocation
     * @param drivebasePose
     * @return run command that calculates the angle and passes it to
     *         setAngleFieldRelative()
     */
    
    public Command aimAtFieldPose(Supplier<Translation2d> targetLocation, Supplier<Pose2d> drivebasePose) {
        return setAngleFieldRelative(new Supplier<Rotation2d>() {
                public Rotation2d get() {
                    Translation2d targetRobotRelative = targetLocation.get().minus(drivebasePose.get().getTranslation());
                    Rotation2d angleFieldRelative = Rotation2d.fromRadians(Math.atan2(targetRobotRelative.getY(), targetRobotRelative.getX()));
                    return angleFieldRelative.plus(Rotation2d.k180deg);
                };
            },
            () -> drivebasePose.get().getRotation());
    }

    public Command aimAtHub() {
        return aimAtFieldPose(() -> POI.HUB1.get().getTranslation(), robotPose);
    }

    /**
     * sets the turret angle to a given field relative angle
     * 
     * @param targetFieldRelativeAngle
     * @param drivebaseAngle
     * @return setAngle Command with the given field relative ange converted into
     *         robot relative
     */
    public Command setAngleFieldRelative(Supplier<Rotation2d> targetFieldRelativeAngle,Supplier<Rotation2d> drivebaseAngle) {
        return setAngle(() -> targetFieldRelativeAngle.get().minus(drivebaseAngle.get()));
    }

    public Command driveToHome() {
        return Commands.sequence(
                setVoltage(() -> TurretConstants.kHomingDrive)
                        .until(TriggerUtil.debounce(() -> getSupplyCurrent().gt(TurretConstants.kHomingCurrentThreshold), TurretConstants.kHomingTime)),
                resetEncoder()).withTimeout(2.0)
                .andThen(setVoltage(Volts.zero()))
                .onlyIf(isIntakeDeployed);
    }

    protected Angle toAngle(Rotation2d angle) {
        return BaseUnits.AngleUnit.of(MathUtil.inputModulus(angle.getMeasure().baseUnitMagnitude(), UnitUtil.CW_180.baseUnitMagnitude(), UnitUtil.CCW_180.baseUnitMagnitude()));
    }

    /**no allocations: returns the parameter or a constant*/
    public Angle clampToHardLimits(Angle angle) {
        return UnitUtil.clamp(angle, TurretConstants.kCWSoftLimit, TurretConstants.kCCWSoftLimit);
    }

    /**no allocations: returns the parameter or a constant*/
    public Angle clampToStowedLimits(Angle angle) {
        return UnitUtil.clamp(angle, TurretConstants.kStowedAngleMin, TurretConstants.kStowedAngleMax);
    }

    /**no allocations: returns the parameter or a constant*/
    public Angle applyDynamicLimits(Angle angle) {
        if (isIntakeDeployed.getAsBoolean()) {
            return clampToHardLimits(angle);
        } else {
            return clampToStowedLimits(angle);
        }
    }
    
    public Supplier<Angle> applyDynamicLimits(Supplier<Angle> angle) {
        return () -> applyDynamicLimits(angle.get());
    }
    
    @Logged
    public boolean atSetpoint() {
        var angle = setpoint.getMeasure();
        return angle.isNear(getAngle(), TurretConstants.kShootTolerance);
    }

    @Logged
    public boolean isInDeadzone() {
        Angle angle = setpoint.getMeasure();
        return angle.lt(TurretConstants.kCWHardLimit) || angle.gt(TurretConstants.kCCWHardLimit);
    }

    
}
