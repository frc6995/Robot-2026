package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodS extends SubsystemBase{

public class hoodConstants {

    public static final Angle kCW = Degrees.of(12.5);
    public static final Angle kCCW = Degrees.of(40);

    public static final Angle kFuelIntakeAngle = Degrees.of(0);
    public static final Angle kStowAngle = Degrees.of(12.5);

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final AngularVelocity kVelocity = DegreesPerSecond.of(0);
    public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(0);
    public static final double kStatorCurrentLimit = 40;
    public static final double kSupplyCurrentLimit = 25;
    public static final double kReduction = 40;
    public static final boolean kMotorInverted = false;
    public static final int kCANID = 42;
    public static final Distance kArmLength = Inches.of(1);
    public static final Mass kArmMass = Pounds.of(0.05);

    public static final double kSimP = 50;
    public static final double kSimI = 0;
    public static final double kSimD = 0;
    public static final double kSimS = 0;
    public static final double kSimG = 0;
    public static final double kSimV = 0;
    public static final double kSimA = 0;
    public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(90);
    public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(45);
}

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(hoodConstants.kP, hoodConstants.kI, hoodConstants.kD, 
  hoodConstants.kVelocity, 
  hoodConstants.kAcceleration)
  .withSimClosedLoopController(hoodConstants.kSimP, hoodConstants.kSimI, hoodConstants.kSimD, 
  hoodConstants.kSimVelocity, 
  hoodConstants.kSimAcceleration)
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(hoodConstants.kS, hoodConstants.kG, hoodConstants.kV, hoodConstants.kA))
  .withSimFeedforward(new ArmFeedforward(hoodConstants.kSimS, hoodConstants.kSimG, hoodConstants.kSimV, hoodConstants.kSimA))
  // Telemetry name and verbosity level
  .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(hoodConstants.kReduction)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(hoodConstants.kMotorInverted)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(hoodConstants.kStatorCurrentLimit))
  .withSupplyCurrentLimit(Amps.of(hoodConstants.kSupplyCurrentLimit));

  private TalonFX motor = new TalonFX(hoodConstants.kCANID);

  private SmartMotorController talonSmartMotorController = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), smcConfig);

  private ArmConfig hoodCfg = new ArmConfig(talonSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(hoodConstants.kCW, hoodConstants.kCCW)
  // Hard limit is applied to the simulation.
  .withHardLimit(hoodConstants.kCW, hoodConstants.kCCW)
  // Starting position is where your arm starts
  .withStartingPosition(hoodConstants.kStowAngle)
  // Length and mass of your arm for sim.
  .withLength(hoodConstants.kArmLength)
  .withMass(hoodConstants.kArmMass)
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private Arm hood = new Arm(hoodCfg);

  public Command setAngle(Angle angle) {return hood.setAngle(angle);}

  public Command set(double dutycycle) { return hood.set(dutycycle);}

  public Command sysId() { return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  public HoodS() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    hood.simIterate();  
  }

}
