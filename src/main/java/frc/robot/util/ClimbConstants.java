package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.crypto.KeyGenerator;
import javax.security.auth.kerberos.KerberosCredMessage;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimbConstants {
    public static class OuterClimbConstants {
        // =======Pivot=======

        // PID constants
        public static final double kPAngle = 50;
        public static final double KIAngle = 0;
        public static final double kDAngle = 0;
        // Feedforward Constants
        public static final double kSAngle = 0;
        public static final double kVAngle = 0;
        public static final double kGAngle = 10;
        public static final double kAAngle = 0;
        // Sim PID Constants
        public static final double kSimPAngle = 50;
        public static final double kSimIAngle = 0;
        public static final double kSimDAngle = 0;
        // Sim FeedFoward Constants
        public static final double kSimSAngle = 0;
        public static final double kSimGAngle = 10;
        public static final double kSimVAngle = 0;
        public static final double kSimAAngle = 0;

        // =======Extension=======

        public static final double kPExtension = 50;
        public static final double KIExtension = 0;
        public static final double kDExtension = 0;
        // Feedforward Constants
        public static final double kSExtension = 50;
        public static final double kGExtension = 0;
        public static final double kVExtension = 10;
        public static final double kAExtension = 0;
        // Sim PID Constants
        public static final double kSimPExtension = 10;
        public static final double kSimIExtension = 0;
        public static final double kSimDExtension = 0;
        // Sim FeedFoward Constants
        public static final double kSimSExtension = 0;
        public static final double kSimGExtension = 0;
        public static final double kSimVExtension = 0;
        public static final double kSimAExtension = 0;

        // CAN IDs
        public static final int kOuterMotorCANID = 51;
        public static final int kInnerMotorCANID = 52;
        public static final int k1FollowMotorCANID = 53;
        // Motor Config Constants
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(180);
        public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(180);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(90);
        public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(90);
        public static final boolean kInvertLeadMotor = true;
        public static final boolean kInvertFollowMotor = false;
        public static final double kSupplyCurrentLimit = 40;
        public static final double kStatorCurrentLimit = 80;
        public static final double kMechCircumference = 1;
        public static final double kReduction = 5;
        public static final double kMinHeight = 5;
        public static final double kMaxHeight = 10;
        public static final double kMOI = 1;
        public static final Distance kLength = Inches.of(10);
        public static final Angle kLowerLimit = Degrees.of(0);
        public static final Angle kUpperLimit = Degrees.of(100);
        public static final Angle kStartingAngle = Degrees.of(00);
        // Sim Constants
        public static final double kHeight = 1;
        public static final double kMass = 100;
        // Setpoints
        public static final Distance kFullExtension = Inches.of(5);
        public static final Distance kL1 = Inches.of(1);
        public static final Distance kL2 = Inches.of(2);
        public static final Distance kL3 = Inches.of(3);
        public static final Angle kL1PrepAngle = Degrees.of(30);

        public static final ClimbExtensionConstantsRecord kExtensionConstants = new ClimbExtensionConstantsRecord(
                kPExtension,
                KIExtension,
                kDExtension,
                kSExtension,
                kGExtension,
                kVExtension,
                kAExtension,
                kSimPExtension,
                kSimIExtension,
                kSimDExtension,
                kSimSExtension,
                kSimGExtension,
                kSimVExtension,
                kSimAExtension,
                kOuterMotorCANID,
                kInnerMotorCANID,
                k1FollowMotorCANID,
                kInvertLeadMotor,
                kInvertFollowMotor,
                kSupplyCurrentLimit,
                kStatorCurrentLimit,
                kMechCircumference,
                kReduction,
                kMinHeight,
                kMaxHeight,
                kHeight,
                kMass,
                kFullExtension,
                kL1,
                kL2,
                kL3);

        public static final ClimbPivotConstantsRecord kPivotConstants = new ClimbPivotConstantsRecord(
                kPAngle,
                KIAngle,
                kDAngle,
                kVelocity,
                kAcceleration,
                kSAngle,
                kGAngle,
                kVAngle,
                kAAngle,
                kSimPAngle,
                kSimIAngle,
                kSimDAngle,
                kSimVelocity,
                kSimAcceleration,
                kSimSAngle,
                kSimGAngle,
                kSimVAngle,
                kSimAAngle,
                kSupplyCurrentLimit,
                kStatorCurrentLimit,
                kInvertLeadMotor,
                kOuterMotorCANID,
                kInnerMotorCANID,
                k1FollowMotorCANID,
                kMOI,
                kLength,
                kReduction,
                kLowerLimit,
                kUpperLimit,
                kStartingAngle,
                kL1PrepAngle);
    }

    public static class InnerClimbConstants {
        // =======Pivot=======

        // =======Pivot=======

        // PID constants
        // PID constants
        public static final double kPAngle = 50;
        public static final double KIAngle = 0;
        public static final double kDAngle = 0;
        // Feedforward Constants
        public static final double kSAngle = 0;
        public static final double kVAngle = 0;
        public static final double kGAngle = 10;
        public static final double kAAngle = 0;
        // Sim PID Constants
        public static final double kSimPAngle = 20;
        public static final double kSimIAngle = 0;
        public static final double kSimDAngle = 0;
        // Sim FeedFoward Constants
        public static final double kSimSAngle = 0;
        public static final double kSimGAngle = 0;
        public static final double kSimVAngle = 0;
        public static final double kSimAAngle = 0;

        // =======Extension=======

        public static final double kPExtension = 5;
        public static final double KIExtension = 0;
        public static final double kDExtension = 0;
        // Feedforward Constants
        public static final double kSExtension = 0;
        public static final double kGExtension = 0;
        public static final double kVExtension = 0;
        public static final double kAExtension = 0;
        // Sim PID Constants
        public static final double kSimPExtension = 20;
        public static final double kSimIExtension = 0;
        public static final double kSimDExtension = 0;
        // Sim FeedFoward Constants
        public static final double kSimSExtension = 0;
        public static final double kSimGExtension = 0;
        public static final double kSimVExtension = 0;
        public static final double kSimAExtension = 0;

        // CAN IDs
        public static final int kOuterMotorCANID = 51;
        public static final int kInnerMotorCANID = 52;
        public static final int k1FollowMotorCANID = 53;
        // Motor Config Constants
        public static final AngularVelocity kVelocity = DegreesPerSecond.of(180);
        public static final AngularVelocity kSimVelocity = DegreesPerSecond.of(30);
        public static final AngularAcceleration kAcceleration = DegreesPerSecondPerSecond.of(90);
        public static final AngularAcceleration kSimAcceleration = DegreesPerSecondPerSecond.of(15);
        public static final boolean kInvertLeadMotor = false;
        public static final boolean kInvertFollowMotor = false;
        public static final double kSupplyCurrentLimit = 40;
        public static final double kStatorCurrentLimit = 80;
        public static final double kMechCircumference = 4;
        public static final double kReduction = 50;
        public static final double kMinHeight = 2; // inches
        public static final double kMaxHeight = 10; // inches
        public static final double kMOI = 0.05;
        public static final Distance kLength = Inches.of(10);
        public static final Angle kLowerLimit = Degrees.of(0);
        public static final Angle kUpperLimit = Degrees.of(100);
        public static final Angle kStartingAngle = Degrees.of(00);
        // Sim Constants
        public static final double kHeight = 2;
        public static final double kMass = 1;
        // Setpoints
        public static final Distance kFullExtension = Inches.of(10);
        public static final Distance kL1 = Inches.of(1);
        public static final Distance kL2 = Inches.of(2);
        public static final Distance kL3 = Inches.of(3);
        public static final Angle kL1PrepAngle = Degrees.of(30);
        public static final Angle kL1ReadyAngle = Degrees.of(0);
        public static final Angle kTolerance = Degrees.of(5);

        public static final ClimbExtensionConstantsRecord kExtensionConstants = new ClimbExtensionConstantsRecord(
                kPExtension,
                KIExtension,
                kDExtension,
                kSExtension,
                kGExtension,
                kVExtension,
                kAExtension,
                kSimPExtension,
                kSimIExtension,
                kSimDExtension,
                kSimSExtension,
                kSimGExtension,
                kSimVExtension,
                kSimAExtension,
                kOuterMotorCANID,
                kInnerMotorCANID,
                k1FollowMotorCANID,
                kInvertLeadMotor,
                kInvertFollowMotor,
                kSupplyCurrentLimit,
                kStatorCurrentLimit,
                kMechCircumference,
                kReduction,
                kMinHeight,
                kMaxHeight,
                kHeight,
                kMass,
                kFullExtension,
                kL1,
                kL2,
                kL3);

        public static final ClimbPivotConstantsRecord kPivotConstants = new ClimbPivotConstantsRecord(
                kPAngle,
                KIAngle,
                kDAngle,
                kVelocity,
                kAcceleration,
                kSAngle,
                kGAngle,
                kVAngle,
                kAAngle,
                kSimPAngle,
                kSimIAngle,
                kSimDAngle,
                kSimVelocity,
                kSimAcceleration,
                kSimSAngle,
                kSimGAngle,
                kSimVAngle,
                kSimAAngle,
                kSupplyCurrentLimit,
                kStatorCurrentLimit,
                kInvertLeadMotor,
                kOuterMotorCANID,
                kInnerMotorCANID,
                k1FollowMotorCANID,
                kMOI,
                kLength,
                kReduction,
                kLowerLimit,
                kUpperLimit,
                kStartingAngle,
                kL1PrepAngle);
    }

    public record ClimbExtensionConstantsRecord(
            double kPExtension,
            double KIExtension,
            double kDExtension,
            double kSExtension,
            double kGExtension,
            double kVExtension,
            double kAExtension,
            // Sim PID Constants
            double kSimPExtension,
            double kSimIExtension,
            double kSimDExtension,
            // Sim FeedFoward Constants
            double kSimSExtension,
            double kSimGExtension,
            double kSimVExtension,
            double kSimAExtension,
            // CAN IDs
            int kOuterMotorCANID,
            int kInnerMotorCANID,
            int k1FollowMotorCANID,
            // Motor Config Constants
            boolean kInvertLeadMotor,
            boolean kInvertFollowMotor,
            double kSupplyCurrentLimit,
            double kStatorCurrentLimit,
            double kMechCircumference,
            double kReduction,
            double kMinHeight,
            double kMaxHeight,
            // Sim Constants
            double kHeight,
            double kMass,
            // Setpoints
            Distance kFullExtension,
            Distance kL1,
            Distance kL2,
            Distance kL3) {
    }

    public record ClimbPivotConstantsRecord(
            // Profiled PID Constants
            double kPAngle,
            double KIAngle,
            double kDAngle,
            AngularVelocity kVelocity,
            AngularAcceleration kAcceleration,
            // Feeforward Constants
            double kSAngle,
            double kGAngle,
            double kVAngle,
            double kAAngle,
            // Sim Profiled PID
            double kSimPAngle,
            double kSimIAngle,
            double kSimDAngle,
            AngularVelocity kSimVelocity,
            AngularAcceleration kSimAcceleration,
            // Sim Feedforward
            double kSimSAngle,
            double kSimGAngle,
            double kSimVAngle,
            double kSimAAngle,
            // Motor Configs
            double kSupplyCurrentLimit,
            double kStatorCurrentLimit,
            boolean kMotorInverted,
            int kOuterMotorCANID,
            int kInnerMotorCANID,
            int k1FollowMotorCANID,
            // Physical Properties
            double kMOI,
            Distance kLength,
            double kReduction,
            // Setpoints and Stops
            Angle kLowerLimit,
            Angle kUpperLimit,
            Angle kStartingAngle,
            Angle kL1PrepAngle) {

    }

}
