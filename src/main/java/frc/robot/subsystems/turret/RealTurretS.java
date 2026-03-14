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
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import static edu.wpi.first.math.util.Units.radiansToRotations;
import static edu.wpi.first.math.util.Units.rotationsToRadians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.turret.RealTurretS.TurretConstants;
import frc.robot.util.POI;
// import frc.robot.util.RobotVisualizer;
import frc.robot.util.UnitUtil;
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
        public static int kCANID = 51;
        public static final double kReduction = 12.5;
        public static double kP = 0;
        public static double kI = 0.0;
        public static double kD = 0.2;
        public static double kS = 0.3;
        public static double kG = 0.0;
        public static double kV = rotationsToRadians(0.12 * kReduction);
        public static double kA = 0.0;
        public static AngularVelocity kVelocity = DegreesPerSecond.of(720.0);
        public static AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(1500.0);

        public static Angle kCWHardLimit = Degrees.of(-138.64);
        public static Angle kCCWHardLimit = Degrees.of(187.6);
                public static Angle kCWSoftLimit =kCWHardLimit;
        public static Angle kCCWSoftLimit = kCCWHardLimit;
        public static Angle kStowedAngle = kCWHardLimit;
        public static Angle kStartAngle = kStowedAngle;
        public static Angle kTolerance = Degrees.of(5);
        public static Angle kStowedAngleMin = kStowedAngle.minus(kTolerance);
        public static Angle kStowedAngleMax = kStowedAngle.plus(kTolerance);


        public static final double kStatorLimit = 80.0;
        public static final double kSupplyLimit = 40.0;

        public static final Voltage kHomingDrive = Volts.of(-1.0);
        public static final Current kHomingCurrentThreshold = Amps.of(1.0);
        public static final double kHomingTime = 0.33;

        public static boolean kIsInverted = false;

        // WIP
        public static MomentOfInertia kMOI = Units.KilogramSquareMeters.of(0.1); // kg*m^2
        public static Distance kRadius = Units.Inches.of(3.1875); // Radius of the Turret from center

    }

    private final TalonFX m_turretMotor = new TalonFX(TurretConstants.kCANID, TunerConstants.kHigherBus);
    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD,
                    TurretConstants.kVelocity,
                    TurretConstants.kAcceleration)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(TurretConstants.kReduction)))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("TurretMotor", RobotContainer.kTelemetryVerbosity)
            .withStatorCurrentLimit(Amps.of(TurretConstants.kStatorLimit))
            .withSupplyCurrentLimit(Amps.of(TurretConstants.kSupplyLimit))
            .withMotorInverted(TurretConstants.kIsInverted)
            .withFeedforward(
                    new ArmFeedforward(TurretConstants.kS, TurretConstants.kG, TurretConstants.kV, TurretConstants.kA))

            .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController turretMotorSMC = new TalonFXWrapper(m_turretMotor, DCMotor.getKrakenX44(1),
            motorConfig);

    private final PivotConfig m_config = new PivotConfig(turretMotorSMC)
            .withHardLimit(TurretConstants.kCWSoftLimit, TurretConstants.kCCWSoftLimit)
            // .withSoftLimits(TurretConstants.kCWLimit.plus(Degrees.of(10)),
            // TurretConstants.kCCWLimit.minus(Degrees.of(10)))
            .withTelemetry("Turret", RobotContainer.kTelemetryVerbosity)
            .withStartingPosition(TurretConstants.kStartAngle)
            // .withWrapping(TurretConstants.kCWLimit, TurretConstants.kCCWLimit)

            // WIP
            .withMOI(Units.Inches.of(3.1875), Pounds.of(14.245));

    private final Pivot m_turret = new Pivot(m_config);

    public RealTurretS(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds,
            BooleanSupplier isIntakeDeployed) {
        super(robotPose, robotSpeeds, isIntakeDeployed);
    }

    @Override
    public void periodic() {
        double currentAngleRad = m_turret.getAngle().in(Radians);
        // RobotVisualizer.updateTurret(currentAngleRad);
       // m_turret.updateTelemetry();

    }

    @Override
    public void simulationPeriodic() {
        m_turret.simIterate();
    }

    public Command setAngle(Supplier<Rotation2d> angle) {
        // Command setangle = Commands.run(() -> setAngle(angle.get()));
        // setangle.addRequirements(this);
        // return setangle;
        return m_turret.setAngle(() -> applyDynamicLimits(toAngle(angle.get())));
    }

    public Command setAngle(Rotation2d angle) {
        return m_turret.setAngle(applyDynamicLimits(toAngle(angle)));
    }

    // private PositionVoltage positionPid = new PositionVoltage(0).withVelocity(0);

    // public Command setAngle(Rotation2d angle) {
    //     return this.run(()->m_turretMotor.setControl(positionPid.withPosition(applyDynamicLimits(toAngle(angle))).withVelocity(radiansToRotations(Math.PI))));//radiansToRotations(-robotSpeeds.get().omegaRadiansPerSecond))));
    // }

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
        return Commands.runOnce(() -> turretMotorSMC.setVoltage(voltage));
    }

    public Command sysId() {
        return m_turret.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }

    public Angle getAngle() {
        return m_turret.getAngle();
    }

    public Optional<Angle> getSetpoint() {
        return m_turret.getMechanismSetpoint();
    }

    public Current getSupplyCurrent() {
        return m_turretMotor.getSupplyCurrent().getValue();
    }

    public Command resetEncoder() {
        return runOnce(() -> turretMotorSMC.setEncoderPosition(TurretConstants.kCWHardLimit)).ignoringDisable(true);
    }
}
