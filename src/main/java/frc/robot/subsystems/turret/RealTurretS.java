package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.util.POI;
import frc.robot.util.RobotVisualizer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RealTurretS extends TurretS {
    public static class TurretConstants {
        public static int kCANID = 41;

        public static double kP = 0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kS = 0.0;
        public static double kG = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;
        public static AngularVelocity kVelocity = DegreesPerSecond.of(180.0);
        public static AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(360.0);

        public static double kSimP = 30;
        public static double kSimI = 0.0;
        public static double kSimD = 0.0;
        public static double kSimS = 0.0;
        public static double kSimG = 0.0;
        public static double kSimV = 4.0;
        public static double kSimA = 0.0;
        public static AngularVelocity kSimVelocity = DegreesPerSecond.of(180.0);
        public static AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(360.0);
        public static Rotation2d kStowedAngle = Rotation2d.fromDegrees(90);

        
        public static Rotation2d kCWLimit = Rotation2d.fromDegrees(-165);
        public static Rotation2d kCCWLimit = Rotation2d.fromDegrees(165);
        public static Rotation2d kStartAngle = kCCWLimit;
        public static Rotation2d kTolerance = Rotation2d.fromDegrees(5);

        public static double kReduction = 50.0;
        public static double kStatorLimit = 80.0;
        public static double kSupplyLimit = 40.0;

        public static Voltage kHomingDrive = Volts.of(-1.0);
        public static double kHomingCurrentThreshold = 3.0;

        public static boolean kIsInverted = false;

        // WIP
        public static MomentOfInertia kMOI = Units.KilogramSquareMeters.of(0.1); // kg*m^2
        public static Distance kRadius = Units.Inches.of(3.1875); // Radius of the Turret from center

    }

    private Supplier<Pose2d> robotPose;
    private Supplier<ChassisSpeeds> robotSpeeds;
    private Supplier<Boolean> isIntakeDeployed;

    public RealTurretS(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds, Supplier<Boolean> isIntakeDeployed) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.isIntakeDeployed = isIntakeDeployed;
    }

    private final TalonFX m_turretMotor = new TalonFX(TurretConstants.kCANID, TunerConstants.kCANBus);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD,
                    TurretConstants.kVelocity,
                    TurretConstants.kAcceleration)
            .withSimClosedLoopController(TurretConstants.kSimP, TurretConstants.kSimI, TurretConstants.kSimD,
                    TurretConstants.kSimVelocity,
                    TurretConstants.kSimAcceleration)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(TurretConstants.kReduction)))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(TurretConstants.kStatorLimit))
            .withSupplyCurrentLimit(Amps.of(TurretConstants.kSupplyLimit))
            .withMotorInverted(TurretConstants.kIsInverted)
            .withFeedforward(
                    new ArmFeedforward(TurretConstants.kS, TurretConstants.kG, TurretConstants.kV, TurretConstants.kA))
            .withSimFeedforward(
                    new ArmFeedforward(TurretConstants.kSimS, TurretConstants.kSimG, TurretConstants.kSimV,
                            TurretConstants.kSimA))            .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController turretMotorSMC = new TalonFXWrapper(m_turretMotor, DCMotor.getKrakenX44(1),
            motorConfig);

    private final PivotConfig m_config = new PivotConfig(turretMotorSMC)
            .withHardLimit(TurretConstants.kCWLimit.getMeasure(), TurretConstants.kCCWLimit.getMeasure())
            // .withSoftLimits(TurretConstants.kCWLimit.plus(Degrees.of(10)),
            //         TurretConstants.kCCWLimit.minus(Degrees.of(10)))
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withStartingPosition(TurretConstants.kStartAngle.getMeasure())
          //  .withWrapping(TurretConstants.kCWLimit, TurretConstants.kCCWLimit)
            
            // WIP
            .withMOI(Units.Inches.of(3.1875), Pounds.of(14.245));
            
    private final Pivot m_turret = new Pivot(m_config);

    @Override
    public void periodic() {
        double currentAngleRad = m_turret.getAngle().in(Radians);
        RobotVisualizer.updateTurret(currentAngleRad);
        m_turret.updateTelemetry();
        
    }

    @Override
    public void simulationPeriodic() {
        m_turret.simIterate();
    }

    public Command setAngle(Supplier<Rotation2d> angle) {
        return m_turret.setAngle(() -> rotationClamp(angle.get(), TurretConstants.kCWLimit, TurretConstants.kCCWLimit).getMeasure());
    }

    public Command setAngle(Rotation2d angle) {
        return m_turret.setAngle(angle.getMeasure());
    }

    public Command setVoltage(Supplier<Voltage> voltageSupplier) {
        return m_turret.setVoltage(voltageSupplier);
    }

    /**
     * Sets the turret motor voltage.
     * 
     * @param voltage (as a double)
     * @return
     */
    public Command setVoltage(Voltage voltage) {
        return m_turret.setVoltage(voltage);
    }

    public Command sysId() {
        return m_turret.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }


    public Supplier<Rotation2d> applyDynamicLimits(Supplier<Rotation2d> angle) {
        return () -> isIntakeDeployed.get() ? rotationClamp(angle.get(), TurretConstants.kCWLimit, TurretConstants.kCCWLimit) : rotationClamp(angle.get(), TurretConstants.kStowedAngle.minus(TurretConstants.kTolerance), TurretConstants.kStowedAngle.plus(TurretConstants.kTolerance));
    }

    public Angle getAngle() {
        return m_turret.getAngle();
    }

    public Optional<Angle> getSetpoint() {
        return m_turret.getMechanismSetpoint();
    }

    public boolean atSetpoint() {
        var refOpt = getSetpoint();
        return refOpt.isPresent() && refOpt.get().isNear(getAngle(), TurretConstants.kTolerance.getMeasure());
    }

    public Current getCurrent() {
        return m_turretMotor.getSupplyCurrent().getValue();
    }

    public Command resetEncoder() {
        return runOnce(() -> turretMotorSMC.setEncoderPosition(
                Degrees.of(0))).ignoringDisable(true);
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
                    return angleFieldRelative;
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
                setVoltage(TurretConstants.kHomingDrive)
                        .until(() -> getCurrent().in(Amps) > TurretConstants.kHomingCurrentThreshold),
                this.runOnce(() -> m_turretMotor.setPosition(Degrees.of(0))).ignoringDisable(true))
                .andThen(setVoltage(Volts.of(0))).withTimeout(2.0);
    }

    private Rotation2d rotationClamp(Rotation2d value, Rotation2d min, Rotation2d max) {
        return Rotation2d.fromDegrees(MathUtil.clamp(value.getDegrees(), min.getDegrees(), max.getDegrees()));
    }
}
