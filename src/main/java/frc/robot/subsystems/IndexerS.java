package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
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
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IndexerS extends SubsystemBase {
    public class IndexerConstants {
        public static final int kCAN_ID = 31; 
       

        public static final int kStatorCurrentLimit = 120; 
        public static final int kSupplyCurrentLimit = 80; 
        public static final int kGearRatio = 50;


        public static final int kP = 0;
        public static final int kI = 0; 
        public static final int kD = 0;

        public static final AngularVelocity kVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(0);


        public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(0);


        public static final int kSimKP = 0; 
        public static final int kSimKI = 0;
        public static final int kSimKD = 0;

        public static final int kS = 0;
        public static final int kSimS = 0;
      
         public static final boolean kInverted = false;


    }
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(IndexerConstants.kP,IndexerConstants.kI, IndexerConstants.kD, IndexerConstants.kVelocity, IndexerConstants.kAcceleration)
            .withSimClosedLoopController(IndexerConstants.kSimKP,IndexerConstants.kSimKI, IndexerConstants.kSimKD, IndexerConstants.kSimVelocity, IndexerConstants.kSimAcceleration)
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(IndexerConstants.kS,0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(IndexerConstants.kSimS,0,0))
            // Telemetry name and verbosity level
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example GearBox.fromReductionStages(3,4) is the same as
            // GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to
            // your motor.
            // You could also use .withGearing(12) which does the same thing.
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(IndexerConstants.kGearRatio)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(IndexerConstants.kInverted)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(IndexerConstants.kStatorCurrentLimit))
            .withSupplyCurrentLimit(Amps.of(IndexerConstants.kSupplyCurrentLimit))
            .withExternalEncoder(new CANcoder(IndexerConstants.kCAN_ID))
            .withUseExternalFeedbackEncoder(true);
        
        private TalonFX indexerMotor = new TalonFX(IndexerConstants.kCAN_ID);
        private SmartMotorController indexerMotorController= new TalonFXWrapper(indexerMotor, DCMotor.getKrakenX44(1),smcConfig) ;

        public Command setVoltage(Voltage volts) {  
            return Commands.runOnce(() -> indexerMotorController.setVoltage(volts));
        }

        public Current getCurrent() {
            var currentOptional = indexerMotorController.getSupplyCurrent();

            return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
        }


}
