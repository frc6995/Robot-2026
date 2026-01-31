
package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;

public class IntakeRollerS extends SubsystemBase{
    public class rollerConstants {

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kVelocity = 0;
    public static final double kAcceleration = 0;
    public static final int kReduction = 3;
    public static final int kCANID = 22;
    public static final double kStatorCurrentLimit = 100;
    public static final boolean kMotorInverted = false;

    public static final double kSimP = 0;
    public static final double kSimI = 0;
    public static final double kSimD = 0;
    public static final double kSimS = 0;
    public static final double kSimG = 0;
    public static final double kSimV = 0;
    public static final double kSimA = 0;
    public static final double kSimVelocity = 0;
    public static final double kSimAcceleration = 0;

    public static final Voltage kIntakeVoltage = Volts.of(5);
    
    }

private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.OPEN_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(rollerConstants.kP, rollerConstants.kI, rollerConstants.kD, 
  DegreesPerSecond.of(rollerConstants.kVelocity), 
  DegreesPerSecondPerSecond.of(rollerConstants.kAcceleration))
  .withSimClosedLoopController(rollerConstants.kSimP, rollerConstants.kSimI, rollerConstants.kSimD,
  DegreesPerSecond.of(rollerConstants.kSimVelocity), 
  DegreesPerSecondPerSecond.of(rollerConstants.kSimAcceleration))
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(rollerConstants.kS, rollerConstants.kV, rollerConstants.kA))
  .withSimFeedforward(new SimpleMotorFeedforward(rollerConstants.kS, rollerConstants.kV, rollerConstants.kA))
  // Telemetry name and verbosity level
  .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(rollerConstants.kReduction)))  // Motor properties to prevent over currenting.
  .withMotorInverted(rollerConstants.kMotorInverted)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(rollerConstants.kStatorCurrentLimit));

  private TalonFX motor = new TalonFX(rollerConstants.kCANID);

  private SmartMotorController talonFXSmartMotorController = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig); 

 public Command setVoltage(Voltage volts) {
        return Commands.runOnce(() -> motor.setVoltage(volts.in(Volts)));
    }

    public Current getCurrent() {
        return motor.getSupplyCurrent().getValue();
    }
  
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    talonFXSmartMotorController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    talonFXSmartMotorController.simIterate();
  }

  

public IntakeRollerS() {

}
}


