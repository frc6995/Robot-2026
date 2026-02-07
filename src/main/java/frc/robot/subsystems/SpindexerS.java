package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotVisualizer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * A class to control the primary rotating component of the Dye Rotor
 */
public class SpindexerS extends SubsystemBase {
    public class SpindexerConstants {
        // CAN IDs
        public static final int kCANID = 32;
        // Profiled PID Constants
        public static final int kP = 0;
        public static final int kI = 0;
        public static final int kD = 0;
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(0);
        // Sim Profiled PID Constants
        public static final int kSimKP = 0;
        public static final int kSimKI = 0;
        public static final int kSimKD = 0;
        public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(0);
        // Feedforward Constants
        public static final double kS = 0;
        public static final double kSimS = 0;
        // Motor Properties
        public static final int kStatorCurrentLimit = 120;
        public static final int kSupplyCurrentLimit = 80;
        public static final int kGearRatio = 50;
        public static final boolean kInverted = false;
        // Sim Constants
        public static final Mass kMass = Pounds.of(0);
        public static final Distance kRadius = Inches.of(0);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0);
    }

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            // PID Constants
            .withClosedLoopController(SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD,
                    SpindexerConstants.kVelocity, SpindexerConstants.kAcceleration)
            .withSimClosedLoopController(SpindexerConstants.kSimKP, SpindexerConstants.kSimKI,
                    SpindexerConstants.kSimKD, SpindexerConstants.kSimVelocity, SpindexerConstants.kSimAcceleration)

            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(SpindexerConstants.kS, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(SpindexerConstants.kSimS, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(SpindexerConstants.kGearRatio)))
            // Motor Properties
            .withMotorInverted(SpindexerConstants.kInverted)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(SpindexerConstants.kStatorCurrentLimit))
            .withSupplyCurrentLimit(Amps.of(SpindexerConstants.kSupplyCurrentLimit));

    // Motor Object
    private TalonFX m_spindexerMotor = new TalonFX(SpindexerConstants.kCANID);
    // SmartMotorController Object
    private SmartMotorController m_spindexerController = new TalonFXWrapper(m_spindexerMotor, DCMotor.getKrakenX44(1),
            smcConfig);

    /**
     * Sends a specified voltage to the spindexer motor.
     * 
     * @param volts The number of volts
     * @return A {@link edu.wpi.first.wpilibj2.command.Command Command} to send the
     *         specified voltage to the motor.
     */
    public Command setVoltage(Supplier<Voltage> volts) {
        return Commands.runOnce(() -> m_spindexerController.setVoltage(volts.get()));
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

    public Command setVelocity(Supplier<AngularVelocity> angularVelocity) {
        return Commands.runOnce(() -> m_spindexerController.setVelocity(angularVelocity.get()));
    }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngleRad = m_spindexerMotor.getPosition().getValue().in(Radians);
    RobotVisualizer.updateSpindexer(currentAngleRad);
    m_spindexerController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_spindexerController.simIterate();
  }

}
