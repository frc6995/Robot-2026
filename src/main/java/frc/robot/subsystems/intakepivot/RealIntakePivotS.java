
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.RobotVisualizer;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RealIntakePivotS extends IntakePivotS {
  public class IntakePivotConstants {
      // CAN IDs
    public static final int kCANID = 31;
      // Profiled PID Constants
    public static final double kP = 56;
    public static final double kI = 0;
    public static final double kD = 0.2;
    public static final AngularVelocity kVelocity = DegreesPerSecond.of(2880);
    public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(1440);
      // Feeforward Constants
    public static final double kS = 0;
    public static final double kG = 1.210;
    public static final double kV = 0.928;
    public static final double kA = 0.16;
      // Sim Profiled PID
    public static final double kSimP = 56;
    public static final double kSimI = 0;
    public static final double kSimD = 0.2;
    public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(2880);
    public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(1440);
      // Sim Feedforward
    public static final double kSimS = 0;
    public static final double kSimG = 1.210;
    public static final double kSimV = 0.928;
    public static final double kSimA = 0.16;
      // Motor Configs
    public static final double kSupplyCurrentLimit = 80;
    public static final double kStatorCurrentLimit = 120;
      // Physical Properties
    public static final double kMOI = 0.05;
    public static final Distance kLength = Inches.of(5.6);
    public static final double kReduction = 57.5;
      // Setpoints and Stops
    public static final Angle kLowerLimit = Degrees.of(0);
    public static final Angle kUpperLimit = Degrees.of(120);
    public static final Angle kFuelIntakeAngle= kLowerLimit;
    public static final Angle kStowAngle = kUpperLimit;
    public static final Angle kTolerance = Degrees.of(0);
  }

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD,
      IntakePivotConstants.kVelocity, 
      IntakePivotConstants.kAcceleration)
      // can be seperate for sim:
      .withSimClosedLoopController(IntakePivotConstants.kSimP, IntakePivotConstants.kSimI, IntakePivotConstants.kSimD,
          IntakePivotConstants.kSimVelocity,
          IntakePivotConstants.kSimAcceleration)
      // Feedforward Constants
      .withFeedforward(
          new ArmFeedforward(IntakePivotConstants.kS, IntakePivotConstants.kG, IntakePivotConstants.kV, IntakePivotConstants.kA))
      .withSimFeedforward(
          new ArmFeedforward(IntakePivotConstants.kSimS, IntakePivotConstants.kSimG, IntakePivotConstants.kSimV, IntakePivotConstants.kSimA))
      // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(IntakePivotConstants.kReduction)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(IntakePivotConstants.kStatorCurrentLimit));

  // Vendor motor controller object
  private TalonFX intakePivotMotor = new TalonFX(IntakePivotConstants.kCANID, TunerConstants.kCANBus);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController IntakeSMC = new TalonFXWrapper(intakePivotMotor, DCMotor.getKrakenX60(1), smcConfig);

  private ArmConfig intakeCfg = new ArmConfig(IntakeSMC)
      // Soft limit is applied to the SmartMotorControllers PID

      .withHardLimit(IntakePivotConstants.kLowerLimit, IntakePivotConstants.kUpperLimit)
      // Starting position is where your arm starts
      .withStartingPosition(IntakePivotConstants.kStowAngle)

      // Length and mass of your arm for sim.
      .withLength(IntakePivotConstants.kLength)

      .withMOI(IntakePivotConstants.kMOI)

      // Telemetry name and verbosity for the arm.
      .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm intakePivot = new Arm(intakeCfg);

  /**
   * Set the angle of the arm.
   * 
   * @param angle Angle to go to.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return intakePivot.setAngle(angle);
  }

  public Command setAngle(Angle angle) {
    return intakePivot.setAngle(angle);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return intakePivot.setVoltage(volts);
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  public Command resetEncoder() {
        return runOnce(() -> IntakeSMC.setEncoderPosition(
                Degrees.of(0))).ignoringDisable(true);
    }

  @Override
  public void periodic() {
    super.periodic();
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    intakePivot.simIterate();
  }

  public Angle getAngle() {
    return intakePivot.getAngle();
  }

  public boolean isIntakeDeployed() {
    return intakePivot.getAngle().isNear(IntakePivotConstants.kFuelIntakeAngle, IntakePivotConstants.kTolerance);
  }
}