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

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;


import yams.mechanisms.velocity.FlyWheel;
import edu.wpi.first.math.Pair;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;



public class ClimbS extends SubsystemBase {
    public static class ClimbConstants {
     // PID Constants 
    public static final double kP = 0;
    public static final double KI = 0;
    public static final double kD = 0; 
     // Feedforward Constants 
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
     // Sim PID Constants
    public static final double kSimP = 0;
    public static final double kSimI = 0;
    public static final double kSimD = 0;
    //Sim FeedFoward Constants 
    public static final double kSimS = 0;
    public static final double kSimV = 0;
    public static final double kSimA = 0;
    // CAN IDs
    public static final int kLeadMotorCANID = 51;
    public static final int k1FollowMotorCANID = 52;
    public static final int k2FollowMotorCANID = 53;
     // Motor Config Constants 
    public static final boolean kInvertLeadMotor = true;
    public static final boolean kInvertFollowMotor = false;
    public static final double kSupplyCurrentLimit = 40; 
    public static final double kStatorCurrentLimit = 80;
     // Sim Constants
    public static final double kHeight = 1;
    public static final double kMass = 100;
     // Setpoints 
     public static final AngularVelocity kFullExtension = RotationsPerSecond.of(4400.0/60.0);
     public static final AngularVekocity kL1 = RotationsPerSecond.of(4400.0/60.0);
     public static final AngularVelocity kL2 = RotationsPerSecond.of (4400/60.0);
     public static final AngularVelocity kL3 = RotationsPerSecond.of(4400/60.0);
     
    }

}
