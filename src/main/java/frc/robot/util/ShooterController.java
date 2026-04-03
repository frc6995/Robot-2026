package frc.robot.util;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.concurrent.Flow.Publisher;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.RealFlyWheelS.FlywheelConstants;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

// Inspiration taken from 6328
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
    static double timeFudge = -0.2;
    private static final double[][] kTimeOfFlightData = {
        {1.23, 1.356+timeFudge},
        {2.5, 1.0347966805},
        {3.64, 1.186+timeFudge},
        {4.44, 1.164+timeFudge}
    };

    private static final double HOOD_MIN = HoodConstants.kLowerLimit.in(Degrees);
    private static final double HOOD_MAX = HoodConstants.kUpperLimit.in(Degrees);

    private static boolean inTower = false;

    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final double LATENCY_SECONDS = 0.03; // adjust later

    private static ShooterController instance = null;
    private ShooterTargetData cachedData = new ShooterTargetData(Rotation2d.kZero, 0,HOOD_MIN);

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();


    private final Supplier<Pose2d> robotPose;
    private final Supplier<ChassisSpeeds> robotSpeeds;
    private final Supplier<ChassisSpeeds> lastSpeeds;
    private final Function<Pose2d, Pose2d> goalPose;

    public NetworkTable goalPoseTable; 
    public StructPublisher<Pose2d> targetPosePub;
    public StructPublisher<Pose2d> projectedPosePub;
    public DoublePublisher distanceToTargetPub;

    private ShooterController(
        Supplier<Pose2d> robotPose,
        Supplier<ChassisSpeeds> robotSpeeds,
        Supplier<ChassisSpeeds> lastSpeeds,
        Function<Pose2d, Pose2d> goalPose
    ) {
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
        this.lastSpeeds = lastSpeeds;
        this.goalPose = goalPose;

        goalPoseTable = NetworkTableInstance.getDefault().getTable("Aim");
        targetPosePub = goalPoseTable.getStructTopic("target", Pose2d.struct).publish();
        projectedPosePub = goalPoseTable.getStructTopic("projected", Pose2d.struct).publish();
        distanceToTargetPub = goalPoseTable.getDoubleTopic("distance").publish();

        populateLUTs();
    }

    public static void initialize(Supplier<SwerveDriveState> currentState, Supplier<SwerveDriveState> lastState, Function<Pose2d, Pose2d> targetPose) {
        if(instance == null) {
            Function<ChassisSpeeds, ChassisSpeeds> speedsFieldRelative = (ch) -> ChassisSpeeds.fromRobotRelativeSpeeds(ch, currentState.get().Pose.getRotation());
            instance = new ShooterController(
                () -> currentState.get().Pose,
                () -> speedsFieldRelative.apply(currentState.get().Speeds),
                () -> speedsFieldRelative.apply(lastState.get().Speeds),
                targetPose
            );
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
            return POI.bottomPassingPoint.get();
        }
        else if (POI.centerZone.get().contains(drivePose.getTranslation())) {
            if (POI.centerZone.get().getCenter().getY() > drivePose.getY())
                return POI.topPassingPoint.get();
            else
                return POI.bottomPassingPoint.get();
        }
        else {
            if (POI.allianceZone.get().contains(drivePose.getTranslation())) {
                if (POI.towerZone.get().contains(drivePose.getTranslation()))
                    inTower = true;     else inTower = false;
                return POI.HUB1.get();
            }
            else {
                return POI.HUB1.get();
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
        ChassisSpeeds lastSpeeds = this.lastSpeeds.get();
        

        Pose2d estimatedPose = currentPose.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * LATENCY_SECONDS,
                speeds.vyMetersPerSecond * LATENCY_SECONDS,
                speeds.omegaRadiansPerSecond * LATENCY_SECONDS
            )
        );

        Translation2d goalTranslation = goalPose.apply(estimatedPose).getTranslation();
        Translation2d delta = goalTranslation.minus(estimatedPose.getTranslation());
        
        double distance = goalTranslation.getDistance(estimatedPose.getTranslation());
        double timeOfFlight = tofMap.get(distance);
        // double timeOfFlight = tofFunction.apply(distance);

        boolean isLongRange = distance > 5.5;

        if(isLongRange) {
            updateTelemetry(goalTranslation, delta, distance);
            cachedData = new ShooterTargetData(
                delta.getAngle().minus(estimatedPose.getRotation()).plus(Rotation2d.k180deg),
                rpmMap.get(distance),
                hoodMap.get(distance)
            );
            return cachedData;
        }
        else if (inTower) {
            updateTelemetry(goalTranslation, delta, distance);
            cachedData = new ShooterTargetData(
                delta.getAngle().minus(estimatedPose.getRotation()).plus(Rotation2d.k180deg),
                FlywheelConstants.kInTowerRPM,
                HoodConstants.kInTowerAngle
            );
            return cachedData;
        }
        
        Translation2d projectedTranslation = estimatedPose.getTranslation();
        delta = goalTranslation.minus(projectedTranslation);
        distance = delta.getNorm();

        for(int i = 0; i < 20; i++) {
            timeOfFlight = tofMap.get(distance);
            // timeOfFlight = tofFunction.apply(distance);
            projectedTranslation = estimatedPose.getTranslation().plus(new Translation2d(
                speeds.vxMetersPerSecond * timeOfFlight,
                speeds.vyMetersPerSecond * timeOfFlight
            ));
            delta = goalTranslation.minus(projectedTranslation);
            distance = delta.getNorm();
        }

        Rotation2d turretFieldAngle = delta.getAngle();

        Rotation2d turretRobotAngle =
            turretFieldAngle.minus(estimatedPose.getRotation()).plus(Rotation2d.k180deg);

        double finalRPM = rpmMap.get(distance);

        double finalHood =
            MathUtil.clamp(
                hoodMap.get(distance),
                HOOD_MIN,
                HOOD_MAX
            );

        cachedData.hoodAngleDeg = finalHood;
        cachedData.rpm = finalRPM;
        cachedData.turretAngle = turretRobotAngle;

        updateTelemetry(goalTranslation, projectedTranslation, distance);

        return cachedData;
    }

    private void updateTelemetry(Translation2d goal, Translation2d projectedPose, double distance) {
        // if(RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0)
        targetPosePub.accept(new Pose2d(goal, Rotation2d.kZero));
        projectedPosePub.accept(new Pose2d(projectedPose, Rotation2d.kZero));
        distanceToTargetPub.accept(distance);
    }
}


