package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IndexerS extends SubsystemBase {
    public class IndexerConstants {
            // CAN IDs
        public static final int kCAN_ID = 31; 
            // Motor Properties
        public static final boolean kInverted = false;
        public static final int kStatorCurrentLimit = 120; 
        public static final int kSupplyCurrentLimit = 80; 
        public static final int kGearRatio = 5;
    }
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.OPEN_LOOP)
            // Apply Telemetry Config
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            // Motor Physicsal Properties
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(IndexerConstants.kGearRatio)))
        .withMotorInverted(IndexerConstants.kInverted)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(IndexerConstants.kStatorCurrentLimit))
        .withSupplyCurrentLimit(Amps.of(IndexerConstants.kSupplyCurrentLimit));

    private TalonFX m_indexerMotor = new TalonFX(IndexerConstants.kCAN_ID);
    private SmartMotorController m_indexerController = new TalonFXWrapper(m_indexerMotor, DCMotor.getKrakenX44(1),smcConfig);

    public Command setVoltage(Voltage volts) {  
        return Commands.runOnce(() -> m_indexerController.setVoltage(volts));
    }

    public Current getCurrent() {
        var currentOptional = m_indexerController.getSupplyCurrent();

        return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
    }
}
