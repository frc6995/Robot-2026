package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Telemetry;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * A class to control the primary rotating component of the Dye Rotor
 */
public class RealSpindexerS extends SpindexerS {
    public class SpindexerConstants {
        // CAN IDs
        public static final int kCANID = 62;
        // Profiled PID Constants
        public static final double kP = 4.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(900);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(90000000);
        public static final Voltage kVoltage = Volts.of(4);
        // Feedforward Constants
        public static final double kS = 0;
        // Motor Properties
        public static final int kStatorCurrentLimit = 120;
        public static final int kSupplyCurrentLimit = 10;
        public static final int kGearRatio = 50;
        public static final boolean kInverted = true;
        // Sim Constants
        public static final Mass kMass = Pounds.of(1);
        public static final Distance kDiameter = Inches.of(4);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.1);
    }

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            // PID Constants
            .withClosedLoopController(SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD,
                    SpindexerConstants.kVelocity, SpindexerConstants.kAcceleration)

            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(SpindexerConstants.kS, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("SpindexerMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(SpindexerConstants.kGearRatio)))
            // Motor Properties
            .withMotorInverted(SpindexerConstants.kInverted)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(SpindexerConstants.kStatorCurrentLimit))
            .withSupplyCurrentLimit(Amps.of(SpindexerConstants.kSupplyCurrentLimit));

    // Motor Object
    private TalonFX m_spindexerMotor = new TalonFX(SpindexerConstants.kCANID, TunerConstants.kCANBus);
    // SmartMotorController Object
    private SmartMotorController m_spindexerController = new TalonFXWrapper(m_spindexerMotor, DCMotor.getKrakenX44(1),
            smcConfig);

            private final FlyWheelConfig spindexerConfig = new FlyWheelConfig(m_spindexerController)
            .withDiameter(SpindexerConstants.kDiameter)
            .withMass(SpindexerConstants.kMass)
            .withUpperSoftLimit(RPM.of(1000))
            .withTelemetry("Spindexer", TelemetryVerbosity.HIGH);

            private FlyWheel m_spindexer = new FlyWheel(spindexerConfig);

    /**
     * Sends a specified voltage to the spindexer motor.
     * 
     * @param volts The number of volts
     * @return A {@link edu.wpi.first.wpilibj2.command.Command Command} to send the
     *         specified voltage to the motor.
     */
    public Command setVoltage(Supplier<Voltage> voltage) {
        return runOnce(() -> m_spindexerController.setVoltage(voltage.get()));
    }

    /**
     * Retrieves the supply current of the motor if present.
     * 
     * @return The current of the motor, or -1 if not present.
     */
    public Current getCurrent() {
        var currentOptional = m_spindexerController.getSupplyCurrent();

        return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
    }

    public Command getVelocity() {return m_spindexer.run(m_spindexer.getSpeed());}

    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return runOnce(() -> m_spindexerController.setVelocity(speed.get()));
    }

    public Command resetEncoder() {
        return runOnce(() -> m_spindexerController.setEncoderPosition(
                Degrees.zero())).ignoringDisable(true);
    }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_spindexer.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_spindexer.simIterate();
  }

}
