package frc.robot.subsystems.vision.detection;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.detection.RealODVision.ODVisionConstants;

public abstract class ObjectVision {
    protected ArrayList<GamePiece> gamePieces = new ArrayList<GamePiece>();
    protected Supplier<Pose2d> robotPose;

    protected record GamePiece(Translation2d translation, double detectionTime) {}

    protected ObjectVision(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
    }

    public abstract void update();

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

        // Based off algorithm from FRC 1678
    protected static Translation2d getRobotToObject(double tx, double ty) {
        double totalAngleY = Units.degreesToRadians(-ty) - ODVisionConstants.kCameraOffset.getRotation().getY();
        Distance distAwayY = ODVisionConstants.kCameraOffset.getMeasureZ().minus((ODVisionConstants.kGamePieceDiameter.div(2)).div(Math.tan(totalAngleY)));
        // Distance distAwayY = Meters.of(
        //     (VisionConstants.LL_OFFSETS[0].getMeasureZ().in(Meters) - (ODVisionConstants.kGamePieceDiameter.in(Meters) / 2.0)) / Math.tan(totalAngleY)
        // );

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