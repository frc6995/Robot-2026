package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Telemetry;
import frc.robot.util.ClimbConstants.ClimbPivotConstantsRecord;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
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

public class ClimbPivotS extends SubsystemBase {
   private ClimbPivotConstantsRecord m_constants;

   private SmartMotorControllerConfig smcConfig;
   private TalonFX talon;
   private SmartMotorController talonSmartMotorController;
   private ArmConfig armCfg;
   private Arm climbArm;

   public ClimbPivotS(ClimbPivotConstantsRecord constants) {
      this.m_constants = constants;
      smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // feedback
            .withClosedLoopController(constants.kPAngle(), constants.KIAngle(), constants.kDAngle())
            .withSimClosedLoopController(constants.kSimPAngle(), constants.kSimIAngle(), constants.kSimDAngle())
            // feedforward
            .withFeedforward(new ArmFeedforward(constants.kSAngle(), constants.kGAngle(), constants.kVAngle()))
            .withSimFeedforward(new ArmFeedforward(constants.kSimSAngle(), constants.kSimGAngle(), constants.kSimVAngle()))
            // telemetry
            .withTelemetry("Climb Pivot Motor", TelemetryVerbosity.HIGH)
            // gearing
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(constants.kReduction())))
            // motor properities
            .withMotorInverted(constants.kMotorInverted())
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(constants.kStatorCurrentLimit()));

      talon = new TalonFX(constants.kInnerMotorCANID());

      talonSmartMotorController = new TalonFXWrapper(talon, DCMotor.getKrakenX44(1), smcConfig);

      armCfg = new ArmConfig(talonSmartMotorController)
            .withSoftLimits(constants.kLowerLimit(), constants.kUpperLimit())
            .withHardLimit(constants.kLowerLimit(), constants.kUpperLimit())
            .withStartingPosition(constants.kStartingAngle())
            .withLength(constants.kLength())
            .withMOI(constants.kMOI())
            .withTelemetry("Climb Arm", TelemetryVerbosity.HIGH);

      climbArm = new Arm(armCfg);
   }

   public Command setAngle(Supplier<Angle> angle) {
    return climbArm.setAngle(angle);
  }

  public Command setVoltage(Supplier<Voltage> volts) {
    return climbArm.setVoltage(volts);
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return climbArm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run
      climbArm.updateTelemetry();
   }

   @Override
   public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
      climbArm.simIterate();
   }

}
