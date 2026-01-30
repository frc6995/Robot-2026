// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.SmartMotorController;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class FlyWheelS extends SubsystemBase {
  /** Creates a new FlyWheels. */
  public class tuneConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kSimP = 0;
    public static final double kSimI = 0;
    public static final double kSimD = 0;
    public static final double kSimS = 0;
    public static final double kSimV = 0;
    public static final double kSimA = 0;

    public static final int kMotorOneCANID = 43;
    public static final int kMotorTwoCANID = 43;
    public static final double kcurrentLimit = 40;

    public static final double kDiameter = 1;
    public static final double kMass = 1;
    public static final AngularVelocity kMaxSpeed = RotationsPerSecond.of(4400.0 / 60.0);

    public static final Boolean invertedOne = true;
    public static final boolean invertedTwo = false;

    public static final AngularVelocity kVelocity = DegreesPerSecond.of(0);
    public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(0);
  }

  private TalonFX motorOne = new TalonFX(tuneConstants.kMotorOneCANID);

  private TalonFX motorTwo = new TalonFX(tuneConstants.kMotorTwoCANID);
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)

      // PID(Needs Tuning)
      .withClosedLoopController(tuneConstants.kP, tuneConstants.kI, tuneConstants.kD, tuneConstants.kVelocity,
          tuneConstants.kAcceleration)
      .withSimClosedLoopController(tuneConstants.kSimP, tuneConstants.kSimI, tuneConstants.kSimD,
          tuneConstants.kVelocity,
          tuneConstants.kAcceleration)

      .withFeedforward(new SimpleMotorFeedforward(tuneConstants.kS, tuneConstants.kV, tuneConstants.kA))
      .withSimFeedforward(new SimpleMotorFeedforward(tuneConstants.kSimS, tuneConstants.kSimV, tuneConstants.kSimA))

      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)

      // Gear Ratio(Needs tuning)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.25)))

      .withMotorInverted(tuneConstants.invertedOne)

      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of((tuneConstants.kcurrentLimit)))

      .withFollowers(Pair.of(motorTwo, tuneConstants.invertedTwo));

  private SmartMotorController motorOneController = new TalonFXWrapper(motorOne, DCMotor.getKrakenX60(2),
      smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motorOneController)
      .withDiameter(Inches.of(tuneConstants.kDiameter))
      .withMass(Pounds.of(tuneConstants.kMass))
      .withUpperSoftLimit(tuneConstants.kMaxSpeed)
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private FlyWheel shooter = new FlyWheel(shooterConfig);

  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  public Current getCurrent() {
    var currentOptional = shooter.getMotorController().getSupplyCurrent();

    return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
  }

  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setVoltage(Voltage voltage) {
    return shooter.setVoltage(voltage);
  }

  public FlyWheelS() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
