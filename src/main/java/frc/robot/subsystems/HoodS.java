package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.POI;
import frc.robot.util.RobotVisualizer;
import frc.robot.util.TriggerUtil;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodS extends SubsystemBase {
  public class HoodConstants {
      // CAN IDs
    public static final int kCANID = 42;
      // PID-FF Constants
    public static final double kP = 15;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
      // Sim PID-FF Constants
    public static final double kSimP = 15;
    public static final double kSimI = 0;
    public static final double kSimD = 0;
    public static final double kSimS = 0;

      // Setpoints and Limits
    public static final Angle kLowerLimit = Degrees.of(12.5); // CW Limit
    public static final Angle kUpperLimit = Degrees.of(40); // CCW Limit
    public static final Angle kStowAngle = kLowerLimit;
    public static final double[][] kAngleData = {
        // Distance (Meters), Angle(Degrees)
        { 1, 12.5 },
        { 5, 40 },
    };
      // Motor Setup
    public static final double kStatorCurrentLimit = 40;
    public static final double kSupplyCurrentLimit = 25;
    public static final double kReduction = 40;
    public static final boolean kMotorInverted = false;
      // Sim Constants
    public static final Distance kArmLength = Inches.of(1);
    public static final Double kMOI = 0.05;
      // Hood Safety Constants
    public static final Distance kSafetyOverride_NoSpeed = Meters.of(1.5);
    public static final Distance kSafetyOverride_Final = Meters.of(0.5);
    public static final LinearVelocity kSafetyOverrideVelocity = MetersPerSecond.of(0.2);
  }

  public InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
      .withSimClosedLoopController(HoodConstants.kSimP, HoodConstants.kSimI, HoodConstants.kSimD)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(HoodConstants.kS, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(HoodConstants.kSimS, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(HoodConstants.kReduction)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(HoodConstants.kMotorInverted)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(HoodConstants.kStatorCurrentLimit))
      .withSupplyCurrentLimit(Amps.of(HoodConstants.kSupplyCurrentLimit));

  private TalonFX motor = new TalonFX(HoodConstants.kCANID);

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
      .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private Pivot hood = new Pivot(hoodCfg);

  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> robotSpeeds;
  private BooleanSupplier shouldApplyDynamicLimit;

  public HoodS(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds) {
    this.robotPose = robotPose;
    this.robotSpeeds = robotSpeeds;

    this.shouldApplyDynamicLimit = 
      TriggerUtil.or(
        TriggerUtil.and(
          TriggerUtil.isWithinZone(
            () -> new Translation2d(POI.kOriginToTrench.get().minus(HoodConstants.kSafetyOverride_NoSpeed), Meters.of(0)),
            () -> new Translation2d(POI.kOriginToTrench.get().plus(HoodConstants.kSafetyOverride_NoSpeed), Meters.of(AllianceFlipUtil.FIELD_LENGTH)),
            robotPose),
          () -> Math.abs(robotSpeeds.get().vxMetersPerSecond) > HoodConstants.kSafetyOverrideVelocity.in(MetersPerSecond)
        ),
        TriggerUtil.isWithinZone(
          () -> new Translation2d(POI.kOriginToTrench.get().minus(HoodConstants.kSafetyOverride_Final), Meters.of(0)),
          () -> new Translation2d(POI.kOriginToTrench.get().plus(HoodConstants.kSafetyOverride_Final), Meters.of(AllianceFlipUtil.FIELD_LENGTH)),
          robotPose));
    
    for(double[] entry : HoodConstants.kAngleData){
      table.put(entry[0], entry[1]);
      
    }
  }

  public Command setAngle(Supplier<Angle> angle) {
    return hood.setAngle(() -> applyDynamicLimits(angle.get(), robotPose.get()));
  }

  public Command set(Supplier<Double> voltage) {
    return hood.set(voltage);
  }

  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  public Command autoHoodAngle() {
    return setAngle(
        () -> Degrees.of(table.get(robotPose.get().getTranslation().getDistance(POI.HUB1.get().getTranslation()))));
  }

  public Angle applyDynamicLimits(Angle targetAngle, Pose2d robotPose) {
    return clampAngle(targetAngle, HoodConstants.kLowerLimit, shouldApplyDynamicLimit.getAsBoolean() ? HoodConstants.kStowAngle : HoodConstants.kUpperLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngleRad = hood.getAngle().in(Radians);
    RobotVisualizer.updateHood(currentAngleRad);
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    hood.simIterate();
  }


  private static Angle clampAngle(Angle value, Angle low, Angle high) {
    return Degrees.of(
      MathUtil.clamp(
        value.in(Degrees),
        low.in(Degrees),
        high.in(Degrees)
      )
    );
  }
}
