package frc.robot.util;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
public class ShooterController {
    public record ShooterCommand(
    Rotation2d turretAngle,
    double rpm,
    double hoodAngleDeg)
    {}
    
    
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();


    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<Pose2d> goalPose;

    private static final double HOOD_MIN = 12.5;
    private static final double HOOD_MAX = 40.0;

    private static final double RPM_CORRECTION_GAIN = 0.7;   // bias small corrections to RPM
    private static final double HOOD_CORRECTION_GAIN = 0.3;

    private static final double LATENCY_SECONDS = 0.15; // adjust later

    public ShooterController(
        Supplier<Pose2d> robotPose,
        Supplier<ChassisSpeeds> robotSpeeds,
        Supplier<Pose2d> goalPose
    ) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.goalPose = goalPose;

        populateLUTs();
    }
    private void populateLUTs() {

        rpmMap.put(1.0, 2800.0);
        rpmMap.put(2.0, 3200.0);
        rpmMap.put(3.0, 3600.0);
        rpmMap.put(4.0, 4000.0);
        rpmMap.put(5.0, 4400.0);

        hoodMap.put(1.0, 12.5);
        hoodMap.put(2.0, 18.0);
        hoodMap.put(3.0, 23.5);
        hoodMap.put(4.0, 29.0);
        hoodMap.put(5.0, 34.5);

        tofMap.put(1.0, 0.30);
        tofMap.put(2.0, 0.35);
        tofMap.put(3.0, 0.42);
        tofMap.put(4.0, 0.50);
        tofMap.put(5.0, 0.60);
    }
    public ShooterCommand calculate() {

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

        return new ShooterCommand(
            turretRobotAngle,
            finalRPM,
            finalHood
        );
    }
}


