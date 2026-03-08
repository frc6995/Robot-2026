package frc.robot.util;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.concurrent.Flow.Publisher;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.subsystems.flywheel.RealFlyWheelS.FlywheelConstants;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class ShooterController {
    public static class ShooterTargetData {
        public Rotation2d turretAngle;
        public double rpm;
        public double hoodAngleDeg;

        public ShooterTargetData(
            Rotation2d turretAngle,
            double rpm,
            double hoodAngleDeg) {
                this.turretAngle = turretAngle;
                this.rpm = rpm;
                this.hoodAngleDeg = hoodAngleDeg;
            }
    }

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
    private final Function<Pose2d, Pose2d> goalPose;

    public NetworkTable goalPoseTable; 
    public StructPublisher<Pose2d> targetPosePub;

    private ShooterController(
        Supplier<Pose2d> robotPose,
        Supplier<ChassisSpeeds> robotSpeeds,
        Function<Pose2d, Pose2d> goalPose
    ) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.goalPose = goalPose;

        goalPoseTable = NetworkTableInstance.getDefault().getTable("Aim");
        targetPosePub = goalPoseTable.getStructTopic("target", Pose2d.struct).publish();

        populateLUTs();
    }

    public static void initialize(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds, Function<Pose2d, Pose2d> targetPose) {
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

    public static Pose2d getAimLocation(Pose2d drivePose) {
        if (POI.topZone.get().contains(drivePose.getTranslation())) {
            return POI.topAllianceZone.get().getCenter();
        }
        else if (POI.bottomZone.get().contains(drivePose.getTranslation())) {
            return POI.bottomAllianceZone.get().getCenter();
        }
        else if (POI.centerZone.get().contains(drivePose.getTranslation())) {
            if (POI.centerZone.get().getCenter().getY() > drivePose.getY())
                return POI.topAllianceZone.get().getCenter();
            else
                return POI.bottomAllianceZone.get().getCenter();
        }
        else {
            if (POI.allianceZone.get().contains(drivePose.getTranslation())) {
                return POI.HUB1.get();
            }
            else {
                return POI.topAllianceZone.get().getCenter();
            }
        }
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
            new Pose2d(projectedTranslation, currentPose.getRotation().plus(Rotation2d.fromDegrees(speeds.omegaRadiansPerSecond)));

        Translation2d goalTranslation = goalPose.apply(projectedPose).getTranslation();
        Translation2d delta = goalTranslation.minus(projectedPose.getTranslation());

        if(RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0)
            targetPosePub.accept(new Pose2d(goalTranslation, new Rotation2d()));

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
            turretFieldAngle.minus(projectedPose.getRotation()).plus(Rotation2d.k180deg);


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

        cachedData.hoodAngleDeg = finalHood;
        cachedData.rpm = finalRPM;
        cachedData.turretAngle = turretRobotAngle;

        return cachedData;
    }
}


