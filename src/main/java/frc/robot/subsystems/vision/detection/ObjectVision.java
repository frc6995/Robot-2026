package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.detection.RealODVision.ODVisionConstants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public abstract class ObjectVision {
    protected ArrayList<GamePiece> gamePieces = new ArrayList<GamePiece>();
    protected Supplier<Pose2d> robotPose;
    protected Timer timer = new Timer();

    protected StructArrayPublisher<Pose2d> objectPoses;
    protected StructArrayPublisher<Pose2d> clusterPoses;
    protected StructPublisher<Pose2d> averageObjectPose;
    protected StructPublisher<Pose2d> bestObjectPose;

    protected static record GamePiece(Translation2d translation, double detectionTime) {}
    public static class Cluster {
        double sumX = 0.0;
        double sumY = 0.0;
        int count = 0;
        Translation2d center = null;

        public void add(GamePiece piece) {
            sumX += piece.translation.getX();
            sumY += piece.translation.getY();
            count++;
        }

        public Translation2d getCenter() {
            center = new Translation2d(
                sumX / count,
                sumY / count
            );
            return center;
        }

        public double calculateScore(Pose2d robotPose) {
            if(center == null) getCenter();
            return robotPose.getTranslation().getDistance(center) * ODVisionConstants.kDistanceWeight + count * ODVisionConstants.kPieceCountWeight;
        }
    }

    protected ObjectVision(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
        timer.start();

        var table = NetworkTableInstance.getDefault().getTable("Vision/Object-Detection/" + ODVisionConstants.kCameraID);
        objectPoses = table.getStructArrayTopic("Object Poses", Pose2d.struct).publish();
        clusterPoses = table.getStructArrayTopic("Cluster Poses", Pose2d.struct).publish();
        averageObjectPose = table.getStructTopic("Average Pose", Pose2d.struct).publish();
        bestObjectPose = table.getStructTopic("Best Pose", Pose2d.struct).publish();
    }

    public abstract void update();

    protected void updateTelemetry() {
        if(RobotContainer.kTelemetryVerbosity == TelemetryVerbosity.HIGH) {
            objectPoses.accept(getObjectPoses().toArray(new Pose2d[gamePieces.size()]));
            var clusters = getClusters();
            var poses = new ArrayList<Pose2d>(clusters.size());
            clusters.forEach((cluster) -> poses.add(new Pose2d(cluster.getCenter(), Rotation2d.kZero)));
            clusterPoses.accept(poses.toArray(new Pose2d[poses.size()]));
        }
        if(RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0) {
            var avgOpt = getAverageObjectLocation();
            averageObjectPose.accept(avgOpt.isPresent() ? new Pose2d(avgOpt.get(), Rotation2d.kZero) : Pose2d.kZero);

            var bestOpt = getBestObjectLocation();
            bestObjectPose.accept(bestOpt.isPresent() ? new Pose2d(bestOpt.get(), Rotation2d.kZero) : Pose2d.kZero);
        }
    }

    public List<GamePiece> getDetectedObjects() {
        return gamePieces;
    }

    public List<Pose2d> getObjectPoses() {
        ArrayList<Pose2d> poses = new ArrayList<>(gamePieces.size());
        gamePieces.forEach((piece) -> poses.add(new Pose2d(piece.translation, Rotation2d.kZero)));
        return poses;
    }
    
    public Optional<Translation2d> getAverageObjectLocation() {
        if(gamePieces.size() == 0) return Optional.empty();

        double xAvg = 0;
        double yAvg = 0;

        for(GamePiece piece : gamePieces) {
            xAvg += piece.translation.getX();
            yAvg += piece.translation.getY();
        }
        return Optional.of(new Translation2d(xAvg / gamePieces.size(), yAvg / gamePieces.size()));

    }

    public Optional<Translation2d> getBestObjectLocation() {
        if(gamePieces.size() == 0) return Optional.empty();
        GamePiece currentBest = gamePieces.get(0);

        for (GamePiece piece : gamePieces) {
            currentBest = getClosest(currentBest, piece, robotPose.get().getTranslation());
        }

        return Optional.of(currentBest.translation);
    }

    protected List<Cluster> getClusters() {
        ArrayList<Cluster> clusters = new ArrayList<Cluster>();
        
        int n = gamePieces.size();
        boolean[] visited = new boolean[n];

        for(int i = 0; i < n; i++) {
            if(visited[i]) continue;

            Cluster c = new Cluster();
            Deque<Integer> stack = new ArrayDeque<>();
            stack.push(i);
            visited[i] = true;

            while (!stack.isEmpty()) {
                int j = stack.pop();
                var piece = gamePieces.get(j);

                c.add(piece);

                for (int k = 0; k < n; k++) {
                    if (visited[k]) continue;

                    var piece2 = gamePieces.get(k);
                    double dx = piece.translation.getX() - piece2.translation.getX();
                    double dy = piece.translation.getY() - piece2.translation.getY();

                    if (dx*dx + dy*dy < ODVisionConstants.kClusterTolerance) {
                        visited[k] = true;
                        stack.push(k);
                    }
                }
            }

            if(c.count >= 2) {
                clusters.add(c);
            }
        }

        return clusters;
    }

    public Optional<Cluster> getBestCluster() {
        Optional<Cluster> best = Optional.empty();
        for(Cluster cl : getClusters()) {
            if(best.isEmpty() || cl.calculateScore(robotPose.get()) > best.get().calculateScore(robotPose.get())) {
                best = Optional.of(cl);
            }
        }
        return best;
    }

        // Based off algorithm from FRC 1678
    protected static Translation2d getRobotToObject(double tx, double ty) {
        double totalAngleY = Units.degreesToRadians(-ty) - ODVisionConstants.kCameraOffset.getRotation().getY();
        // Distance distAwayY = ODVisionConstants.kCameraOffset.getMeasureZ().minus((ODVisionConstants.kGamePieceDiameter.div(2)).div(Math.tan(totalAngleY)));
        Distance distAwayY = Meters.of(
            (ODVisionConstants.kCameraOffset.getMeasureZ().in(Meters) - (ODVisionConstants.kGamePieceDiameter.in(Meters) / 2.0)) / Math.tan(totalAngleY)
        );

        Distance distHypotenuseYToGround = BaseUnits.DistanceUnit.of(Math.hypot(
				distAwayY.in(BaseUnits.DistanceUnit),
				ODVisionConstants.kCameraOffset
						.getMeasureZ()
						.minus(ODVisionConstants.kGamePieceDiameter.div(2))
						.in(BaseUnits.DistanceUnit)));

        double totalAngleX = Units.degreesToRadians(-tx)
                + ODVisionConstants.kCameraOffset.getRotation().getZ();

        Distance distAwayX = distHypotenuseYToGround.times(Math.tan(totalAngleX)); // robot y

        return new Translation2d(distAwayY, distAwayX);
    }

    protected static GamePiece getClosest(GamePiece one, GamePiece two, Translation2d origin) {
        return one.translation.getDistance(origin) < two.translation.getDistance(origin) ? one : two;
    }
}