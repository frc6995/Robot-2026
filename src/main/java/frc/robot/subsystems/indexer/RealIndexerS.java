package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RealIndexerS extends IndexerS {
    public class IndexerConstants {
            // CAN IDs
        public static final int kCAN_ID = 61; 
            // Motor Properties
        public static final boolean kInverted = true;
        public static final int kStatorCurrentLimit = 120; 
        public static final int kSupplyCurrentLimit = 10; 
        public static final int kGearRatio = 5;
            // Setpoints
        public static final Voltage kSlowVoltage = Volts.of(6.0);

    }
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
            // Apply Telemetry Config
        .withTelemetry("IndexerMotor", RobotContainer.kTelemetryVerbosity)
            // Motor Physicsal Properties
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(IndexerConstants.kGearRatio)))
        .withMotorInverted(IndexerConstants.kInverted)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(IndexerConstants.kStatorCurrentLimit))
        .withSupplyCurrentLimit(Amps.of(IndexerConstants.kSupplyCurrentLimit));

    private TalonFX m_indexerMotor = new TalonFX(IndexerConstants.kCAN_ID, TunerConstants.kHigherBus);
    private SmartMotorController m_indexerController = new TalonFXWrapper(m_indexerMotor, DCMotor.getKrakenX44(1),smcConfig);

    public Command setVoltage(Supplier<Voltage> voltage) {  
        return run(() -> m_indexerController.setVoltage(voltage.get())).finallyDo(() -> m_indexerController.setVoltage(Volts.zero()));
    }

    public Current getCurrent() {
        var currentOptional = m_indexerController.getSupplyCurrent();

        return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
    }

    public Command resetEncoder() {
        return runOnce(() -> m_indexerController.setEncoderPosition(
                Degrees.zero())).ignoringDisable(true);
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_indexerController.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_indexerController.simIterate();


    
  }
}
