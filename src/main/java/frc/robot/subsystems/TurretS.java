package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.ObjectInputFilter.Config;
import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretS extends SubsystemBase {

    public static class TurretConstants {
        public static int kCANID = 41;

        public static double kP = 0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kS = 0.0;
        public static double kG = 0.0;
        public static double kV = 0.0;
        public static double kA = 0.0;
        public static AngularVelocity kVelocity = DegreesPerSecond.of(200.0);
        public static AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(200.0);

        public static double kSimP = 40;
        public static double kSimI = 0.0;
        public static double kSimD = 0.0;
        public static double kSimS = 0.0;
        public static double kSimG = 0.0;
        public static double kSimV = 0.0;
        public static double kSimA = 0.0;
        public static AngularVelocity kSimVelocity = DegreesPerSecond.of(200.0);
        public static AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(200.0);

        public static Angle kStartAngle = Degrees.of(150);
        public static Angle kCWLimit = Degrees.of(-180);
        public static Angle kCCWLimit = kStartAngle;

        public static double kReduction = 50.0;
        public static double kStatorLimit = 80.0;
        public static double kSupplyLimit = 40.0;

        public static boolean kIsInverted = false;

        // WIP
        public static MomentOfInertia kMOI = Units.KilogramSquareMeters.of(0.1); // kg*m^2
        public static Distance kRadius = Units.Inches.of(4.0); // Radius of the Turret from center

    }

    private Debouncer m_currentDebouncer = new Debouncer(1.0, Debouncer.DebounceType.kRising);

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
                            TurretConstants.kSimA))
            .withControlMode(ControlMode.CLOSED_LOOP);
    private final SmartMotorController turretMotorSMC = new TalonFXWrapper(m_turretMotor, DCMotor.getKrakenX44(1),
            motorConfig);

    private final PivotConfig m_config = new PivotConfig(turretMotorSMC)
            .withHardLimit(TurretConstants.kCWLimit, TurretConstants.kCCWLimit)
            .withSoftLimits(TurretConstants.kCWLimit, TurretConstants.kCCWLimit)
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withStartingPosition(TurretConstants.kStartAngle)
            // WIP
            .withMOI(Meters.of(0.25), Pounds.of(4));
    private final Pivot m_turret = new Pivot(m_config);

    @Override
    public void periodic() {
        m_turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_turret.simIterate();
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

    public Command setAngle(Angle angle) {
        return m_turret.setAngle(angle);
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

   

        public Command driveToHome() {
    return Commands.sequence(
      setVoltage(Volts.of(-1.0)).until(()-> getSupplyCurrent().magnitude() > 3),
      this.runOnce(()->m_turretMotor.setPosition(Degrees.of(0))).ignoringDisable(true)
      
    ).withTimeout(10.0).andThen(setVoltage(Volts.of(0)));
  }

}
