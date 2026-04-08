// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.UnitUtil;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.SmartMotorController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class RealFlyWheelS extends FlyWheelS {
    public static class FlywheelConstants {
        // PID Constants
        public static final double kP = 0.70;
        public static final double kI = 0;
        public static final double kD = 0.0;
        // Feedforward Constants
        public static final double kS = 0.25;
        public static final double kV = 0.18;
        public static final double kA = 0.0;
        // CAN IDs
        public static final int kLeadMotorCANID = 53;
        public static final int kFollowMotorCANID = 54;
        // Motor Config Constants
        public static final boolean kInvertLeadMotor = false;
        public static final boolean kInvertFollowMotor = true;
        public static final double kSupplyCurrentLimit = 40;
        public static final double kStatorCurrentLimit = 80;
        public static final double kMaxVoltage = 10;
        public static final double kMinVoltage = 0;
        // Sim Constants
        public static final double kDiameter = 2;
        public static final double kMass = 4.15;
        // Setpoints
        public static final AngularVelocity kMaxSpeed = RPM.of(4400.0);
        public static final AngularVelocity kShootSpeed = RPM.of(3000);
        public static final AngularVelocity kTolerance = RPM.of(100);

        public static final double[][] kShooterData = {
                {0.0, 1750},
                {3.0, 1750},
                {4.0, 1850},
                {5.0, 1950},
                {10, 2500},
                {15.0, 3500}

        };

        public static final double[][] kPassShooterData = {
            {0, 1000},
            {1, 1100},
            {2, 1200},
            {3, 1300},
            {4, 1400},
            {5, 1600},
            {6, 1800},
            {8, 2200},
            {10, 2500},
            {15, 2500}
        };

        // TODO: Tune this!
        public static final double kInTowerRPM = 1850;
    }

    // Motors
    private TalonFX m_leadMotor = new TalonFX(FlywheelConstants.kLeadMotorCANID, TunerConstants.kHigherBus);
    private TalonFX m_followerMotor = new TalonFX(FlywheelConstants.kFollowMotorCANID, TunerConstants.kHigherBus);

    // SmartMotorController Config
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Apply PID constants
            .withClosedLoopController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD)
            // Apply Feedforward constants
            .withFeedforward(
                    new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA))
            // Set Telemetry mode
            .withTelemetry("ShooterMotor", RobotContainer.kTelemetryVerbosity)
            // Gear Ratio(Needs tuning)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.33)))
            // Motor Configs
            .withMotorInverted(FlywheelConstants.kInvertLeadMotor)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of((FlywheelConstants.kStatorCurrentLimit)))
            .withSupplyCurrentLimit(Amps.of(FlywheelConstants.kSupplyCurrentLimit))
            // Follower Motor Setup
            .withFollowers(Pair.of(m_followerMotor, FlywheelConstants.kInvertFollowMotor));

    // SmartMotorController Object
    private SmartMotorController m_motorController = new TalonFXWrapper(m_leadMotor, DCMotor.getKrakenX60(2),
            smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(m_motorController)
            // Physical Properties
            .withDiameter(Inches.of(FlywheelConstants.kDiameter))
            .withMass(Pounds.of(FlywheelConstants.kMass))
            // Maximum Speed
            .withUpperSoftLimit(FlywheelConstants.kMaxSpeed)
            // Telemetry Config
            .withTelemetry("ShooterMech", RobotContainer.kTelemetryVerbosity);

    private FlyWheel m_shooter = new FlyWheel(shooterConfig);

    private Optional<AngularVelocity> setpoint = Optional.empty();

    private BooleanSupplier isIntakeDeployed;

    public RealFlyWheelS(BooleanSupplier isIntakeDeployed) {
        this.isIntakeDeployed = isIntakeDeployed;
        VoltageConfigs voltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(FlywheelConstants.kMaxVoltage)
            .withPeakReverseVoltage(FlywheelConstants.kMinVoltage);
            
        m_leadMotor.getConfigurator().apply(voltageConfigs);
        m_followerMotor.getConfigurator().apply(voltageConfigs);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
       // m_shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        m_shooter.simIterate();
    }

    public Command setVelocity(Supplier<AngularVelocity> speed) {
        return m_shooter.setSpeed(() -> {
            var spd = applyDynamicLimits(speed.get());
            setpoint = Optional.of(spd);
            return spd;
        });
    }

    public Command setVoltage(Supplier<Voltage> voltage) {
        return m_shooter.setVoltage(voltage);
    }

    public AngularVelocity getVelocity() {
        return m_shooter.getSpeed();
    }

    public Optional<AngularVelocity> getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return setpoint.isPresent() && getVelocity().isNear(setpoint.get(), FlywheelConstants.kTolerance);
    }

    public Current getCurrent() {
        var currentOptional = m_shooter.getMotorController().getSupplyCurrent();

        return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
    }

    public Command resetEncoder() {
        return runOnce(() -> m_motorController.setEncoderPosition(Degrees.zero())).ignoringDisable(true);
    }

    private AngularVelocity applyDynamicLimits(AngularVelocity velocity) {
        return !isIntakeDeployed.getAsBoolean() ? RPM.zero() : velocity;
    }

}
