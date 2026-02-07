package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ClimbConstants.ClimbConstantsRecord;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimbExtensionS extends SubsystemBase {
    private ClimbConstantsRecord m_constants;

    private SmartMotorControllerConfig smcConfig;
    private Elevator m_elevator;

    public ClimbExtensionS(ClimbConstantsRecord constants) {
        this.m_constants = constants;
        smcConfig = new SmartMotorControllerConfig(this)
                .withControlMode(ControlMode.CLOSED_LOOP)
                // Mechanism Circumference is the distance traveled by each mechanism rotation
                // converting rotations to meters.
                .withMechanismCircumference(Meters.of(Inches.of(constants.kMechCircumference()).in(Meters) * 22))
                // Feedback Constants (PID Constants)
                .withClosedLoopController(constants.kP(), constants.kI(), constants.kD(),
                        constants.kVelocity(),
                        constants.kAcceleration())
                .withSimClosedLoopController(constants.kSimP(), constants.kSimI(), constants.kSimD(),
                        constants.kSimVelocity(),
                        constants.kSimAcceleration())
                // Feedforward Constants
                .withFeedforward(
                        new ElevatorFeedforward(constants.kS(), constants.kG(), constants.kV(), constants.kA()))
                .withSimFeedforward(new ElevatorFeedforward(constants.kSimS(), constants.kSimG(), constants.kSimV(),
                        constants.kSimA()))
                // Telemetry name and verbosity level
                .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
                // Gearing from the motor rotor to final shaft.
                // In this example GearBox.fromReductionStages(3,4) is the same as
                // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
                // your motor.
                // You could also use .withGearing(12) which does the same thing.
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(constants.kReduction())))
                // Motor properties to prevent over currenting.
                .withMotorInverted(constants.kInvertLeadMotor())
                .withIdleMode(MotorMode.BRAKE)
                .withStatorCurrentLimit(Amps.of(constants.kStatorCurrentLimit()));

        final TalonFX talon = new TalonFX(constants.kLeadMotorCANID());

        TalonFXWrapper talonSmartMotorController = new TalonFXWrapper(talon, DCMotor.getKrakenX44(2), smcConfig);

        ElevatorConfig elevconfig = new ElevatorConfig(talonSmartMotorController)
                .withStartingHeight(Meters.of(constants.kHeight()))
                .withHardLimits(Meters.of(constants.kMinHeight()), Meters.of(constants.kMaxHeight()))
                .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
                .withMass(Pounds.of(constants.kMass()));
        m_elevator = new Elevator(elevconfig);

    }

    
  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { return m_elevator.setHeight(height);}
  
  /**
   * Set the height of the m_elevator and ends the command when reached, but not the closed loop controller.
   * @param angle Distance to go to.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height) { return m_elevator.setHeight(height);}
  
  /**
   * Set the m_elevators closed loop controller setpoint.
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { m_elevator.setHeight(height);}

  /**
   * Run sysId on the {@link m_elevator}
   */
  public Command sysId() { return m_elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        m_elevator.simIterate();
    }

}
