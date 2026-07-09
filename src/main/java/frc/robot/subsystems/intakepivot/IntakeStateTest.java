package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class IntakeStateTest extends SubsystemBase {

    public static class IntakePivotConstants {
        // CAN IDs
        public static final int kCANID = 31;

        // PID Constants
        public static final double kP = 120;
        public static final double kI = 0;
        public static final double kD = 0.12;

        // Motion Magic Constraints
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(1000);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(1000);
        public static final AngularVelocity kSlowVelocity = DegreesPerSecond.of(40);

        // Feedforward Constants
        public static final double kS = 0;
        public static final double kG = 0.28;
        public static final double kV = 6.5;
        public static final double kA = 0.2;

        // Current Limits
        public static final double kSupplyCurrentLimit = 46;
        public static final double kStatorCurrentLimit = 120;

        // Physical Properties
        public static final double kMOI = 0.05;
        public static final Distance kLength = Inches.of(5.6);
        public static final double kReduction = 57.5;

        // Setpoints and Limits
        public static final Angle kLowerLimit = Degrees.of(2);
        public static final Angle kUpperLimit = Degrees.of(120);
        public static final Angle kFuelIntakeAngle = kLowerLimit;
        public static final Angle kStowAngle = kUpperLimit;

        public static final Angle kTolerance = Degrees.of(3);

        // Safety
        public static final double kMaxTempCelsius = 60;
    }

    private final TalonFX intakePivotMotor =
            new TalonFX(IntakePivotConstants.kCANID, TunerConstants.kHigherBus);

    private final DynamicMotionMagicVoltage controlRequest =
            new DynamicMotionMagicVoltage(
                    IntakePivotConstants.kStowAngle,
                    IntakePivotConstants.kVelocity,
                    IntakePivotConstants.kAcceleration
            ).withSlot(0);

    public enum IntakeStates {
        IDLE,
        RETRACTED,
        DEPLOYED
    }

    private IntakeStates activeState = IntakeStates.RETRACTED;
    private IntakeStates lastState = IntakeStates.RETRACTED;

    private Angle setpoint = IntakePivotConstants.kStowAngle;

    public IntakeStateTest() {
        configureMotor();

        // This assumes the intake physically starts stowed when the robot boots.
        intakePivotMotor.setPosition(IntakePivotConstants.kStowAngle);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // This makes the TalonFX report mechanism angle instead of raw motor angle.
        config.Feedback.SensorToMechanismRatio = IntakePivotConstants.kReduction;

        // PID + feedforward constants
        config.Slot0.kP = IntakePivotConstants.kP;
        config.Slot0.kI = IntakePivotConstants.kI;
        config.Slot0.kD = IntakePivotConstants.kD;
        config.Slot0.kS = IntakePivotConstants.kS;
        config.Slot0.kG = IntakePivotConstants.kG;
        config.Slot0.kV = IntakePivotConstants.kV;
        config.Slot0.kA = IntakePivotConstants.kA;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // Default Motion Magic constraints
        config.MotionMagic.MotionMagicCruiseVelocity =
                IntakePivotConstants.kVelocity.in(Rotations.per(edu.wpi.first.units.Units.Second));

        config.MotionMagic.MotionMagicAcceleration =
                IntakePivotConstants.kAcceleration.in(
                        Rotations.per(edu.wpi.first.units.Units.Second).per(edu.wpi.first.units.Units.Second)
                );

        // Current limits
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakePivotConstants.kSupplyCurrentLimit;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakePivotConstants.kStatorCurrentLimit;

        // Motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Software limits
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                IntakePivotConstants.kUpperLimit.in(Rotations);

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                IntakePivotConstants.kLowerLimit.in(Rotations);

        intakePivotMotor.getConfigurator().apply(config);
    }

    public void setState(IntakeStates state) {
        this.activeState = state;
    }

    private void deploy() {
        setpoint = applyLimits(IntakePivotConstants.kFuelIntakeAngle);

        intakePivotMotor.setControl(
                controlRequest
                        .withPosition(setpoint)
                        .withVelocity(IntakePivotConstants.kVelocity)
                        .withAcceleration(IntakePivotConstants.kAcceleration)
        );
    }

    private void retract() {
        setpoint = applyLimits(IntakePivotConstants.kStowAngle);

        intakePivotMotor.setControl(
                controlRequest
                        .withPosition(setpoint)
                        .withVelocity(IntakePivotConstants.kVelocity)
                        .withAcceleration(IntakePivotConstants.kAcceleration)
        );
    }

    private void intakeIdle() {
        // Idle holds the last commanded position.
        // This keeps the arm from falling while still using PID/Motion Magic.
        intakePivotMotor.setControl(
                controlRequest
                        .withPosition(setpoint)
                        .withVelocity(IntakePivotConstants.kSlowVelocity)
                        .withAcceleration(IntakePivotConstants.kAcceleration)
        );
    }

    @Override
    public void periodic() {
        if (intakePivotMotor.getDeviceTemp().getValueAsDouble()
                >= IntakePivotConstants.kMaxTempCelsius) {
            intakePivotMotor.setControl(new NeutralOut());
            return;
        }

        if (activeState != lastState) {
            if (activeState == IntakeStates.IDLE) {
                // When entering idle, hold the current angle instead of moving somewhere new.
                setpoint = getAngle();
            }

            lastState = activeState;
        }

        switch (activeState) {
            case IDLE:
                intakeIdle();
                break;

            case RETRACTED:
                retract();
                break;

            case DEPLOYED:
                deploy();
                break;

            default:
                intakeIdle();
                break;
        }
    }

    public Angle getAngle() {
        return intakePivotMotor.getPosition().getValue();
    }

    public Angle getSetpoint() {
        return setpoint;
    }

    public boolean atSetpoint() {
        return getAngle().isNear(setpoint, IntakePivotConstants.kTolerance);
    }

    private Angle applyLimits(Angle angle) {
        double angleDeg = angle.in(Degrees);
        double lowerDeg = IntakePivotConstants.kLowerLimit.in(Degrees);
        double upperDeg = IntakePivotConstants.kUpperLimit.in(Degrees);

        if (angleDeg < lowerDeg) {
            return IntakePivotConstants.kLowerLimit;
        }

        if (angleDeg > upperDeg) {
            return IntakePivotConstants.kUpperLimit;
        }

        return angle;
    }

    public Command deployCommand() {
        return runOnce(() -> setState(IntakeStates.DEPLOYED));
    }

    public Command retractCommand() {
        return runOnce(() -> setState(IntakeStates.RETRACTED));
    }

    public Command idleCommand() {
        return runOnce(() -> setState(IntakeStates.IDLE));
    }
}