
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

import java.util.function.BooleanSupplier;
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
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.RobotVisualizer;
import frc.robot.util.UnitUtil;
// import frc.robot.util.RobotVisualizer;
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
        public static final double kP = 120;
        public static final double kI = 0;
        public static final double kD = 0.12;
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(1000);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(1000);
        // Feeforward Constants
        public static final double kS = 0;
        public static final double kG = 0.28;
        public static final double kV = 6.5;
        public static final double kA = 0.2;
        // Sim Profiled PID
        // public static final double kSimP = 20;
        // public static final double kSimI = 0;
        // public static final double kSimD = 0.12;

        // // Sim Feedforward
        // public static final double kSimS = 0;
        // public static final double kSimG = 0.28;
        // public static final double kSimV = 6.83;
        // public static final double kSimA = 0.03;
        // Motor Configs
        public static final double kSupplyCurrentLimit = 30;
        public static final double kStatorCurrentLimit = 120;
        // Physical Properties
        public static final double kMOI = 0.05;
        public static final Distance kLength = Inches.of(5.6);
        public static final double kReduction = 57.5;
        // Setpoints and Stops
        public static final Angle kLowerLimit = Degrees.of(2);
        public static final Angle kUpperLimit = Degrees.of(120);
        public static final Angle kFuelIntakeAngle = kLowerLimit;
        public static final Angle kStowAngle = kUpperLimit;
        public static final Angle kTolerance = Degrees.of(35);

        public static final Angle kSafetyUpperLimit = Degrees.of(90);
    }

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(IntakePivotConstants.kP, IntakePivotConstants.kI, IntakePivotConstants.kD,
                    IntakePivotConstants.kVelocity,
                    IntakePivotConstants.kAcceleration)

            .withFeedforward(
                    new ArmFeedforward(IntakePivotConstants.kS, IntakePivotConstants.kG, IntakePivotConstants.kV,
                            IntakePivotConstants.kA))
            // .withSimFeedforward(
            // new ArmFeedforward(IntakePivotConstants.kSimS, IntakePivotConstants.kSimG,
            // IntakePivotConstants.kSimV, IntakePivotConstants.kSimA))
            // Telemetry name and verbosity level
            .withTelemetry("ArmMotor", RobotContainer.kTelemetryVerbosity)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(IntakePivotConstants.kReduction)))
            .withMotorInverted(true)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(IntakePivotConstants.kStatorCurrentLimit));

    // Vendor motor controller object
    private TalonFX intakePivotMotor = new TalonFX(IntakePivotConstants.kCANID, TunerConstants.kLowerBus);

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
            .withTelemetry("Intake", RobotContainer.kTelemetryVerbosity);

    // Arm Mechanism
    private Arm intakePivot = new Arm(intakeCfg);
    private BooleanSupplier isTurretStowed;
    private Angle setpoint;

    public RealIntakePivotS(BooleanSupplier isTurretStowed) {
        this.isTurretStowed = isTurretStowed;
    }

    /**
     * Set the angle of the arm.
     * 
     * @param angle Angle to go to.
     */
    public Command setAngle(Supplier<Angle> angle) {
        return intakePivot.setAngle(() -> {
            Angle ang = applyDynamicLimits(angle.get());
            setpoint = ang;
            return ang;
        });
    }

    public Command setAngle(Angle angle) {
        Angle withLimits = applyDynamicLimits(angle);
        setpoint = withLimits;
        return intakePivot.setAngle(withLimits);
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
        return runOnce(() -> IntakeSMC.setEncoderPosition(IntakePivotConstants.kStowAngle)).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        // intakePivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        double currentAngleRad = getAngle().in(Radians);
        RobotVisualizer.updateIntake(currentAngleRad);
        intakePivot.simIterate();
    }

    public Angle getAngle() {
        return intakePivot.getAngle();
    }

    private Angle applyDynamicLimits(Angle angle) {
        return isTurretStowed.getAsBoolean() ? 
            UnitUtil.clamp(angle, IntakePivotConstants.kLowerLimit, IntakePivotConstants.kUpperLimit) 
            : UnitUtil.clamp(angle, IntakePivotConstants.kLowerLimit, IntakePivotConstants.kSafetyUpperLimit);
    }

    public boolean isIntakeDeployed() {
        return !getAngle().isNear(IntakePivotConstants.kStowAngle, IntakePivotConstants.kTolerance);
    }
}