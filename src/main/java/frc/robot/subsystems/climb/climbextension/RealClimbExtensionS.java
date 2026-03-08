package frc.robot.subsystems.climb.climbextension;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RobotVisualizer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RealClimbExtensionS extends ClimbExtensionS {
    public class climbConstants {
        public static final double kP = 5;
        public static final double KI = 0;
        public static final double kD = 0;
        // Feedforward Constants
        public static final double kS = 0;
        public static final double kG = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        // Sim PID Constants
        public static final double kSimP = 20;
        public static final double kSimI = 0;
        public static final double kSimD = 0;
        // Sim FeedFoward Constants
        public static final double kSimS = 0;
        public static final double kSimG = 0;
        public static final double kSimV = 0;
        public static final double kSimA = 0;

        // CAN IDs
        public static final int kMotorCANID = 52;
        // Motor Config Constants
        public static final boolean kInvertLeadMotor = false;
        public static final boolean kInvertFollowMotor = false;
        public static final double kSupplyCurrentLimit = 40;
        public static final double kStatorCurrentLimit = 80;
        public static final double kMechCircumference = 4;
        public static final double kReduction = 50;
        public static final double kMinHeight = 2; // inches
        public static final double kMaxHeight = 10; // inches
        // Sim Constants
        public static final double kHeight = 2;
        public static final double kMass = 5;
        // Setpoints
        public static final Distance kFullExtension = Inches.of(10);
        public static final Distance kL1 = Inches.of(1);
    }

    public RealClimbExtensionS() {

    }

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Mechanism Circumference is the distance traveled by each mechanism rotation
            // converting rotations to meters.
            .withMechanismCircumference(
                    Meters.of(Inches.of(climbConstants.kMechCircumference).in(Meters) * 22))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(climbConstants.kP, climbConstants.KI, climbConstants.kD)
            .withSimClosedLoopController(climbConstants.kSimP, climbConstants.kSimI, climbConstants.kSimD)
            // Feedforward Constants
            .withFeedforward(
                    new ElevatorFeedforward(climbConstants.kS, climbConstants.kG, climbConstants.kV, climbConstants.kA))
            .withSimFeedforward(new ElevatorFeedforward(climbConstants.kSimS, climbConstants.kSimG, climbConstants.kSimV, climbConstants.kSimA))
            // Telemetry name and verbosity level
            .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as
            // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
            // your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(climbConstants.kReduction)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(climbConstants.kInvertLeadMotor)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(climbConstants.kStatorCurrentLimit));

    private TalonFX talon = new TalonFX(climbConstants.kMotorCANID);

    private SmartMotorController talonSmartMotorController = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), smcConfig);

    private ElevatorConfig elevconfig = new ElevatorConfig(talonSmartMotorController)
            .withStartingHeight(Inches.of(climbConstants.kHeight))
            .withHardLimits(Inches.of(climbConstants.kMinHeight), Inches.of(climbConstants.kMaxHeight))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(climbConstants.kMass));
    private Elevator m_elevator = new Elevator(elevconfig);

    /**
     * Set the height of the elevator and does not end the command when reached.
     * 
     * @param angle Distance to go to.
     * @return a Command
     */
    public Command setHeight(Supplier<Distance> height) {
        return m_elevator.setHeight(height);
    }

    /**
     * Set the height of the m_elevator and ends the command when reached, but not
     * the closed loop controller.
     * 
     * @param angle Distance to go to.
     * @return A Command
     */
    public Command setHeightAndStop(Supplier<Distance> height) {
        return m_elevator.setHeight(height);
    }

    /**
     * Set the m_elevators closed loop controller setpoint.
     * 
     * @param angle Distance to go to.
     */
    public void setHeightSetpoint(Supplier<Distance> height) {
        m_elevator.setHeight(height);
    }

    public Command resetEncoder() {
        return runOnce(() -> talonSmartMotorController.setEncoderPosition(Meters.of(0))).ignoringDisable(true);
    }

    /**
     * Run sysId on the {@link m_elevator}
     */

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double currentDistanceMeter = m_elevator.getHeight().in(Meters);
        RobotVisualizer.updateExtend(currentDistanceMeter);
        m_elevator.updateTelemetry();
    }

    public void setDefaultCommand() {
        m_elevator.setHeight(() -> Inches.of(8));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        m_elevator.simIterate();
    }

}
