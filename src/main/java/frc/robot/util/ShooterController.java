package frc.robot.util;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.flywheel.RealFlyWheelS.FlywheelConstants;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
public class ShooterController {
    public record ShooterTargetData(
    Rotation2d turretAngle,
    double rpm,
    double hoodAngleDeg)
    {}
    private static final double[][] kTimeOfFlightData = {
        {1.0, 0.30},
        {5.0, 0.60}
    };

    private static final double RPM_CORRECTION_GAIN = 0.7;   // bias small corrections to RPM
    private static final double HOOD_CORRECTION_GAIN = 0.3;
    private static final double HOOD_MIN = HoodConstants.kLowerLimit.in(Degrees);
    private static final double HOOD_MAX = HoodConstants.kUpperLimit.in(Degrees);

    private static final double LATENCY_SECONDS = 0.02; // adjust later

    private static ShooterController instance = null;
    private ShooterTargetData cachedData = new ShooterTargetData(Rotation2d.kZero, 0,HOOD_MIN);

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> goalPose;

    private ShooterController(
        Supplier<Pose2d> robotPose,
        Supplier<ChassisSpeeds> robotSpeeds,
        Supplier<Pose2d> goalPose
    ) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.goalPose = goalPose;

        populateLUTs();
    }

    public static void initialize(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds, Supplier<Pose2d> targetPose) {
        if(instance == null) {
            instance = new ShooterController(robotPose, robotSpeeds, targetPose);
        }
    }

    public static ShooterController getInstance() {
        if(instance == null) {
            throw new NullPointerException("ShooterController has not yet been initialized!");
        }
        return instance;
    }

    public ShooterTargetData getCachedData() {
        return cachedData;
    }

    private void populateLUTs() {

       for(var value : FlywheelConstants.kShooterData) {
            rpmMap.put(value[0], value[1]);
       }

        for(var value : HoodConstants.kAngleData) {
            hoodMap.put(value[0], value[1]);
        }

        for(var value : kTimeOfFlightData) {
            tofMap.put(value[0], value[1]);
        }
    }
    public ShooterTargetData calculate() {

        Pose2d currentPose = robotPose.get();
        ChassisSpeeds speeds = robotSpeeds.get();

        Translation2d projectedTranslation =
            currentPose.getTranslation().plus(
                new Translation2d(
                    speeds.vxMetersPerSecond * LATENCY_SECONDS,
                    speeds.vyMetersPerSecond * LATENCY_SECONDS
                )
            );

        Pose2d projectedPose =
            new Pose2d(projectedTranslation, currentPose.getRotation());

        Translation2d goalTranslation = goalPose.get().getTranslation();
        Translation2d delta = goalTranslation.minus(projectedPose.getTranslation());

        double distance = delta.getNorm();

        double baseRPM = rpmMap.get(distance);
        double baseHood = hoodMap.get(distance);
        double timeOfFlight = tofMap.get(distance);

        double baselineVelocity =
            distance / timeOfFlight; // m/s

        Translation2d shotUnit = delta.div(distance);

        Translation2d shotVelocityVector =
            shotUnit.times(baselineVelocity);

        Translation2d robotVel =
            new Translation2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
            );

        Translation2d correctedVector =
            shotVelocityVector.minus(robotVel);

        double correctedSpeed = correctedVector.getNorm();


        Rotation2d turretFieldAngle =
            new Rotation2d(
                correctedVector.getX(),
                correctedVector.getY()
            );


        Rotation2d turretRobotAngle =
            turretFieldAngle.minus(projectedPose.getRotation());


        double velocityDelta =
            correctedSpeed - baselineVelocity;

        double rpmCorrection =
            velocityDelta * RPM_CORRECTION_GAIN * 100.0; // scale factor placeholder

        double hoodCorrection =
            velocityDelta * HOOD_CORRECTION_GAIN * 2.0; // degrees scaling placeholder

        double finalRPM =
            baseRPM + rpmCorrection;

        double finalHood =
            MathUtil.clamp(
                baseHood + hoodCorrection,
                HOOD_MIN,
                HOOD_MAX
            );

        cachedData = new ShooterTargetData(
            turretRobotAngle,
            finalRPM,
            finalHood
        );

        return cachedData;
    }
}


