package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import limelight.Limelight;

public class RealODVision extends ObjectVision {
    public static class ODVisionConstants {
        public static final String kCameraID = "limelight-front";
        public static final Pose3d kCameraOffset = new Pose3d(
                new Translation3d(Inches.of(-11.25), Inches.of(0), Inches.of(20.5)),
                new Rotation3d(Degrees.of(0), Degrees.of(-2), Degrees.of(0)));
        public static final Distance kGamePieceDiameter = Inches.of(5.91);

        private static final Distance kClusterRadius = Inches.of(20);

        public static final double kClusterTolerance = Math.pow(kClusterRadius.in(Meters), 2);

        public static final double kDistanceWeight = 0;
        public static final double kPieceCountWeight = 0;
    }

    private Limelight camera = new Limelight(ODVisionConstants.kCameraID);

    public RealODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);
    }

    @Override
    public void update() {
        gamePieces.clear();
        var results = camera.getLatestResults();

        if (results.isPresent()) {
            var detectorTargets = results.get().targets_Detector;
            for (var target : detectorTargets) {
                Translation2d targetLocation = getRobotToObject(target.tx_nocrosshair, target.ty_nocrosshair);
                boolean isCloseEnoughToRobot = targetLocation.getDistance(Translation2d.kZero) < 0;
              //  && targetLocation.getDistance(Translation2d.kZero) > 100
                // Distance is in meters
                if (isCloseEnoughToRobot) {
                    gamePieces.add(gamePieceToField(targetLocation, robotPose.get()));
                }
            }
        }

        updateTelemetry();
    }
}
