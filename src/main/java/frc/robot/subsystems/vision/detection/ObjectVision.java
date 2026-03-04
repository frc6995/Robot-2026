package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.BaseUnits;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.detection.RealODVision.ODVisionConstants;
import frc.robot.util.POI;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public abstract class ObjectVision {
    protected ArrayList<Translation2d> gamePieces = new ArrayList<Translation2d>();
    protected Supplier<Pose2d> robotPose;

    protected StructArrayPublisher<Pose3d> objectPoses;
    protected StructArrayPublisher<Pose2d> clusterPoses;
    protected StructPublisher<Pose2d> averageObjectPose;
    protected StructPublisher<Pose2d> bestObjectPose;
    protected StructArrayPublisher<Pose2d> clusterBounds;

    public static class Cluster {
        double sumX = 0.0;
        double sumY = 0.0;
        int count = 0;
        Translation2d center = null;

        public void add(Translation2d piece) {
            sumX += piece.getX();
            sumY += piece.getY();
            count++;
        }

        public Translation2d getCenter() {
            center = new Translation2d(
                    sumX / count,
                    sumY / count);
            return center;
        }

        public double calculateScore(Pose2d robotPose) {
            if (center == null)
                getCenter();
            return robotPose.getTranslation().getDistance(center) * ODVisionConstants.kDistanceWeight
                    + count * ODVisionConstants.kPieceCountWeight;
        }

        public double getPieceCount() {
            return count;
        }
    }

    protected ObjectVision(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;

        var table = NetworkTableInstance.getDefault()
                .getTable("Vision/Object-Detection/" + ODVisionConstants.kCameraID);
        objectPoses = table.getStructArrayTopic("Object Poses", Pose3d.struct).publish();
        clusterPoses = table.getStructArrayTopic("Cluster Poses", Pose2d.struct).publish();
        averageObjectPose = table.getStructTopic("Average Pose", Pose2d.struct).publish();
        bestObjectPose = table.getStructTopic("Best Pose", Pose2d.struct).publish();
        clusterBounds = table.getStructArrayTopic("Cluster Bounds", Pose2d.struct).publish();
    }

    public abstract void update();

    protected void updateTelemetry() {
        if (RobotContainer.kTelemetryVerbosity == TelemetryVerbosity.HIGH) {
            objectPoses.accept(getObjectPoses().toArray(new Pose3d[gamePieces.size()]));
            var clusters = getClusters();
            var poses = new ArrayList<Pose2d>(clusters.size());
            clusters.forEach((cluster) -> poses.add(new Pose2d(cluster.getCenter(), Rotation2d.kZero)));
            clusterPoses.accept(poses.toArray(new Pose2d[poses.size()]));
        }
        if (RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0) {
            var avgOpt = getAverageObjectLocation();
            averageObjectPose.accept(avgOpt.isPresent() ? new Pose2d(avgOpt.get(), Rotation2d.kZero) : Pose2d.kZero);

            var bestOpt = getBestObjectLocation();
            bestObjectPose.accept(bestOpt.isPresent() ? new Pose2d(bestOpt.get(), Rotation2d.kZero) : Pose2d.kZero);
        }
    }

    public List<Translation2d> getDetectedObjects() {
        return gamePieces;
    }

    public List<Pose3d> getObjectPoses() {
        ArrayList<Pose3d> poses = new ArrayList<>(gamePieces.size());
        gamePieces.forEach((piece) -> poses.add(new Pose3d(new Translation3d(piece), Rotation3d.kZero)));
        return poses;
    }

    public Optional<Translation2d> getAverageObjectLocation() {
        if (gamePieces.size() == 0)
            return Optional.empty();

        double xAvg = 0;
        double yAvg = 0;

        for (Translation2d piece : gamePieces) {
            xAvg += piece.getX();
            yAvg += piece.getY();
        }
        return Optional.of(new Translation2d(xAvg / gamePieces.size(), yAvg / gamePieces.size()));

    }

    public Optional<Translation2d> getBestObjectLocation() {
        if (gamePieces.size() == 0)
            return Optional.empty();
        Translation2d currentBest = gamePieces.get(0);

        for (Translation2d piece : gamePieces) {
            currentBest = getClosest(currentBest, piece, robotPose.get().getTranslation());
        }

        return Optional.of(currentBest);
    }

    protected List<Cluster> getClusters() {
        ArrayList<Cluster> clusters = new ArrayList<Cluster>();

        int n = gamePieces.size();
        boolean[] visited = new boolean[n];

        for (int i = 0; i < n; i++) {
            if (visited[i])
                continue;

            Cluster c = new Cluster();
            Deque<Integer> stack = new ArrayDeque<>();
            stack.push(i);
            visited[i] = true;

            while (!stack.isEmpty()) {
                int j = stack.pop();
                var piece = gamePieces.get(j);

                c.add(piece);

                for (int k = 0; k < n; k++) {
                    if (visited[k])
                        continue;

                    var piece2 = gamePieces.get(k);
                    double dx = piece.getX() - piece2.getX();
                    double dy = piece.getY() - piece2.getY();

                    if (dx * dx + dy * dy < ODVisionConstants.kClusterTolerance) {
                        visited[k] = true;
                        stack.push(k);
                    }
                }
            }

            if (c.count >= 2) {
                clusters.add(c);
            }
        }

        return clusters;
    }

    public Optional<Cluster> getBestCluster() {
        return getBestCluster(POI.kFieldBounds);
    }

    public Optional<Cluster> getBestCluster(Rectangle2d bounds) {
        Optional<Cluster> best = Optional.empty();
        for (Cluster cl : getClusters()) {
            if ((best.isEmpty() || cl.calculateScore(robotPose.get()) > best.get().calculateScore(robotPose.get()))
                    && bounds.contains(cl.getCenter())) {
                best = Optional.of(cl);
            }
        }
        if (RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0) {
            Pose2d center = bounds.getCenter();
            clusterBounds.accept(new Pose2d[] {
                    new Pose2d(
                            center.getTranslation()
                                    .plus(new Translation2d(bounds.getXWidth() / 2, bounds.getYWidth() / 2)),
                            Rotation2d.kZero),
                    new Pose2d(
                            center.getTranslation()
                                    .plus(new Translation2d(-bounds.getXWidth() / 2, -bounds.getYWidth() / 2)),
                            Rotation2d.kZero),
            });
        }
        return best;
    }

    // Based off algorithm from FRC 1678
    protected static Translation2d getRobotToObject(double tx, double ty) {
        double totalAngleY = Units.degreesToRadians(-ty) - ODVisionConstants.kCameraOffset.getRotation().getY();
        double distAwayY = (ODVisionConstants.kCameraOffset.getMeasureZ().in(Meters)
                - (ODVisionConstants.kGamePieceDiameter.in(Meters) / 2.0)) / Math.tan(totalAngleY);

        double distHypotenuseYToGround = Math.hypot(
                distAwayY,
                ODVisionConstants.kCameraOffset
                        .getMeasureZ()
                        .minus(ODVisionConstants.kGamePieceDiameter.div(2))
                        .in(BaseUnits.DistanceUnit));

        double totalAngleX = Units.degreesToRadians(-tx)
                + ODVisionConstants.kCameraOffset.getRotation().getZ();

        double distAwayX = distHypotenuseYToGround * Math.tan(totalAngleX);

        return new Translation2d(distAwayY, distAwayX);
    }

    protected static Translation2d convertPieceToField(Translation2d pieceRobotRelative, Pose2d robotPose) {
        return robotPose.getTranslation().plus(pieceRobotRelative)
                .rotateAround(robotPose.getTranslation(), robotPose.getRotation());
    }

    protected static Translation2d getClosest(Translation2d one, Translation2d two, Translation2d origin) {
        return one.getDistance(origin) < two.getDistance(origin) ? one : two;
    }
}