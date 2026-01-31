
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivotS extends SubsystemBase {
  public class intakeConstants {

    public static final Angle kCW = Degrees.of(-25);
    public static final Angle kCCW = Degrees.of(146);

    public static final Angle kFuelIntakeAngle= Degrees.of(-25);
    public static final Angle kStowAngle = Degrees.of(146);

    public static final double kP = 56;
    public static final double kI = 0;
    public static final double kD = 0.2;
    public static final double kS = 0;
    public static final double kG = 1.210;
    public static final double kV = 0.928;
    public static final double kA = 0.16;
    public static final AngularVelocity kVelocity = DegreesPerSecond.of(2880);
    public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(1440);
    public static final int kCANID = 40;
    public static final double kSupplyCurrentLimit = 80;
    public static final double kStatorCurrentLimit = 120;
    public static final double kMOI = 0.05;
    public static final Distance kLength = Inches.of(5.6);
    public static final double kReduction = 57.5;

    public static final double kSimP = 56;
    public static final double kSimI = 0;
    public static final double kSimD = 0.2;
    public static final double kSimS = 0;
    public static final double kSimG = 1.210;
    public static final double kSimV = 0.928;
    public static final double kSimA = 0.16;
    public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(2880);
    public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(1440);
  }

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(intakeConstants.kP, intakeConstants.kI, intakeConstants.kD,
      intakeConstants.kVelocity, 
      intakeConstants.kAcceleration)
      // can be seperate for sim:
      .withSimClosedLoopController(intakeConstants.kSimP, intakeConstants.kSimI, intakeConstants.kSimD,
          intakeConstants.kSimVelocity,
          intakeConstants.kSimAcceleration)
      // Feedforward Constants
      .withFeedforward(
          new ArmFeedforward(intakeConstants.kS, intakeConstants.kG, intakeConstants.kV, intakeConstants.kA))
      .withSimFeedforward(
          new ArmFeedforward(intakeConstants.kSimS, intakeConstants.kSimG, intakeConstants.kSimV, intakeConstants.kSimA))
      // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(intakeConstants.kReduction)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(intakeConstants.kStatorCurrentLimit));

  // Vendor motor controller object
  private TalonFX intakePivotMotor = new TalonFX(intakeConstants.kCANID, TunerConstants.kCANBus);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController IntakeSMC = new TalonFXWrapper(intakePivotMotor, DCMotor.getKrakenX60(1), smcConfig);

  private ArmConfig armCfg = new ArmConfig(IntakeSMC)
      // Soft limit is applied to the SmartMotorControllers PID

      .withHardLimit(intakeConstants.kCW, intakeConstants.kCCW)
      // Starting position is where your arm starts
      .withStartingPosition(intakeConstants.kStowAngle)

      // Length and mass of your arm for sim.
      .withLength(intakeConstants.kLength)

      .withMOI(intakeConstants.kMOI)

      // Telemetry name and verbosity for the arm.
      .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /**
   * Set the angle of the arm.
   * 
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {

    return arm.setAngle(angle);
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }

  public Angle getAngle() {
    return arm.getAngle();
  }
}