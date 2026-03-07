package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
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
                //new Translation3d(Inches.of(-11.25), Inches.of(0), Inches.of(20.5)),
                new Translation3d(Centimeter.of(-26.363), Centimeter.of(-7.639), Centimeter.of(54.341)),
                new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-9)));
        public static final Distance kGamePieceDiameter = Inches.of(5.91);

        private static final Distance kClusterRadius = Inches.of(16);
        private static final Distance kValidMaxDetectRadius = Meters.of(3);
        private static final Distance kValidMinDetectRadius = Inches.of(24);

        public static final double kDistanceWeight = -1;
        public static final double kPieceCountWeight = 2;

            // Calculated Constants
        public static final double kClusterTolerance = Math.pow(kClusterRadius.in(Meters), 2);
        public static final double kMaxDetectRadiusMeters = kValidMaxDetectRadius.in(Meters);
        public static final double kMinDetectRadiusMeters = kValidMinDetectRadius.in(Meters);

            // Sim Constants
        public static final double kCameraFOVDegrees = 120;
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
                double detectDist = targetLocation.getDistance(Translation2d.kZero);
                if (detectDist < ODVisionConstants.kMaxDetectRadiusMeters && detectDist > ODVisionConstants.kMinDetectRadiusMeters) {
                    gamePieces.add(convertPieceToField(targetLocation, robotPose.get()));
                }
            }
        }

        updateTelemetry();
    }
}
