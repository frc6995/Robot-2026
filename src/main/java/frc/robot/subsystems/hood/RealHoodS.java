package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import choreo.util.FieldSize;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.POI;
import frc.robot.util.ShooterController;
import frc.robot.util.UnitUtil;
import frc.robot.util.ShooterController.ShooterTargetData;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RealHoodS extends HoodS {
    public class HoodConstants {
        // CAN IDs
        public static final int kCANID = 52;
        // PID-FF Constants
        public static final double kP = 160;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 4.99;
        public static final double kA = 0.04;

        // Setpoints and Limits
        public static final Angle kLowerLimit = Degrees.of(12.5); // CW Limit
        public static final Angle kUpperLimit = Degrees.of(38); // CCW Limit
        public static final Angle kStowAngle = kLowerLimit;
        public static final Angle kTolerance = Degrees.of(2);
        public static final double[][] kAngleData = {
                // Distance (Meters), Angle(Degrees)
                { 1, 12.5 },
                { 2.2, 20 },
                { 4, 35 },
                { 5, 40 },
        };
        // Motor Setup
        public static final double kStatorCurrentLimit = 40;
        public static final double kSupplyCurrentLimit = 25;
        public static final double kReduction = 73.33;
        public static final boolean kMotorInverted = false;
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(1600);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(2700);
        // Sim Constants
        public static final Distance kArmLength = Inches.of(9.384);
        public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.00671959172);
        // Hood Safety Constants
        // public static final Distance kSafetyOverride_NoSpeed = Meters.of(1.5);
        public static final Distance kSafetyOverride_Final = Meters.of(0.153);
        // public static final LinearVelocity kSafetyOverrideVelocity = MetersPerSecond.of(0.2);
        public static final double kHoodRetractTime = 0.25;
    }

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD,
                    HoodConstants.kVelocity,
                    HoodConstants.kAcceleration)
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(HoodConstants.kS, HoodConstants.kV, HoodConstants.kA))
            // Telemetry name and verbosity level
            .withTelemetry("HoodMotor", RobotContainer.kTelemetryVerbosity)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(HoodConstants.kReduction)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(HoodConstants.kMotorInverted)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(HoodConstants.kStatorCurrentLimit))
            .withSupplyCurrentLimit(Amps.of(HoodConstants.kSupplyCurrentLimit));

    private TalonFX motor = new TalonFX(HoodConstants.kCANID, TunerConstants.kHigherBus);

    private SmartMotorController talonSmartMotorController = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1),
            smcConfig);

    private PivotConfig hoodCfg = new PivotConfig(talonSmartMotorController)
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(HoodConstants.kLowerLimit, HoodConstants.kUpperLimit)
            // Hard limit is applied to the simulation.
            .withHardLimit(HoodConstants.kLowerLimit, HoodConstants.kUpperLimit)
            // Starting position is where your arm starts
            .withStartingPosition(HoodConstants.kStowAngle)
            .withMOI(HoodConstants.kMOI)
            // Telemetry name and verbosity for the arm.
            .withTelemetry("Hood", RobotContainer.kTelemetryVerbosity);

    private Pivot hood = new Pivot(hoodCfg);

    private Supplier<Pose2d> robotPose;
    private Supplier<Translation2d> robotTranslation;
    private Supplier<ChassisSpeeds> robotSpeeds;
    private Supplier<ChassisSpeeds> lastSpeeds;
    private double poseEstPeriod = 0.02;

    private BooleanSupplier shouldApplyDynamicLimit;

    public RealHoodS(Supplier<SwerveDriveState> currentState, Supplier<SwerveDriveState> lastState) {
        this.robotPose = () -> currentState.get().Pose;
        this.robotTranslation = () -> robotPose.get().getTranslation();
        this.robotSpeeds = () -> currentState.get().Speeds;
        this.lastSpeeds = () -> lastState.get().Speeds;

        this.shouldApplyDynamicLimit = makeShouldApplyDynamicLimit();
    }

    private BooleanSupplier makeShouldApplyDynamicLimit() {
        Distance halfField = FieldSize.FIELD_WIDTH.div(2);
        Pose2d trenchCenterBlue = new Pose2d(
                new Translation2d(POI.kOriginToTrenchBlue, halfField), Rotation2d.kZero);
        Pose2d trenchCenterRed = new Pose2d(
            new Translation2d(POI.kOriginToTrenchRed, halfField), Rotation2d.kZero);

        var safetyLength = HoodConstants.kSafetyOverride_Final.times(2);

        Rectangle2d trenchBlue = new Rectangle2d(trenchCenterBlue, safetyLength, FieldSize.FIELD_WIDTH);
        Rectangle2d trenchRed = new Rectangle2d(trenchCenterRed, safetyLength, FieldSize.FIELD_WIDTH);
        return () -> {
                    var translation = robotTranslation.get();
                    var projTranslation = CommandSwerveDrivetrain.getProjectedTranslation(translation, robotSpeeds.get(), lastSpeeds.get(), poseEstPeriod, HoodConstants.kHoodRetractTime);
                    return (trenchBlue.contains(translation) || trenchBlue.contains(projTranslation))
                            || (trenchRed.contains(translation) || trenchBlue.contains(projTranslation));
                };
    }

    public Command setAngle(Supplier<Angle> angle) {
        return hood.setAngle(() -> applyDynamicLimits(angle.get(), robotPose.get()));
    }

    public Command setVoltage(Supplier<Voltage> voltage) {
        return hood.setVoltage(voltage);
    }

    public Command sysId() {
        return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    public Command autoHoodAngle() {
        return runSOTF(ShooterController.getInstance()::getCachedData);
    }

    public Angle applyDynamicLimits(Angle targetAngle, Pose2d robotPose) {
        return UnitUtil.clamp(targetAngle, HoodConstants.kLowerLimit,
                shouldApplyDynamicLimit.getAsBoolean() ? HoodConstants.kStowAngle : HoodConstants.kUpperLimit);
    }

    public boolean isHoodSafe() {
        return hood.getAngle().isNear(HoodConstants.kStowAngle, HoodConstants.kTolerance);
    }

    public boolean isHoodReady() {
        var setpoint = getSetpoint();
        return hood.getAngle().isNear(setpoint.isPresent() ? setpoint.get() : HoodConstants.kStowAngle,
                HoodConstants.kTolerance);
    }

    public Command runSOTF(Supplier<ShooterTargetData> dataSupplier) {
        return setAngle(() -> applyDynamicLimits(
                Degrees.of(dataSupplier.get().hoodAngleDeg),
                robotPose.get()));
    }

    public Optional<Angle> getSetpoint() {
        return hood.getMechanismSetpoint();
    }

    public Command resetEncoder() {
        return runOnce(() -> talonSmartMotorController.setEncoderPosition(
                Degrees.zero())).ignoringDisable(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // double currentAngleRad = hood.getAngle().in(Radians);
        // RobotVisualizer.updateHood(currentAngleRad);
        // hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        hood.simIterate();
    }

    
}
