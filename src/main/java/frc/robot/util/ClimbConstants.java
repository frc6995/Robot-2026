package frc.robot.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ClimbConstants {
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
    // Sim FeedFoward Constants
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
    public static final AngularVelocity kFullExtension = RotationsPerSecond.of(4400.0 / 60.0);
    public static final AngularVelocity kL1 = RotationsPerSecond.of(4400.0 / 60.0);
    public static final AngularVelocity kL2 = RotationsPerSecond.of(4400 / 60.0);
    public static final AngularVelocity kL3 = RotationsPerSecond.of(4400 / 60.0);

    public record ClimbConstantsRecord(
            double kP,
            double kI,
            double kD,
            double kS,
            double kG,
            double kV,
            double kA,
            // Sim PID Constants
            double kSimP,
            double kSimI,
            double kSimD,
            // Sim FeedFoward Constants
            double kSimS,
            double kSimG,
            double kSimV,
            double kSimA,
            // CAN IDs
            int kLeadMotorCANID,
            int k1FollowMotorCANID,
            int k2FollowMotorCANID,
            // Motor Config Constants
            boolean kInvertLeadMotor,
            boolean kInvertFollowMotor,
            double kSupplyCurrentLimit,
            double kStatorCurrentLimit,
            double kMechCircumference,
            LinearVelocity kVelocity,
            LinearAcceleration kAcceleration,
            LinearVelocity kSimVelocity,
            LinearAcceleration kSimAcceleration,
            double kReduction,
            double kMinHeight,
            double kMaxHeight,
            // Sim Constants
            double kHeight,
            double kMass,
            // Setpoints
            AngularVelocity kFullExtension,
            AngularVelocity kL1,
            AngularVelocity kL2,
            AngularVelocity kL3) {

    }

}
