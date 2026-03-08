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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Distance;
import limelight.Limelight;
import limelight.results.RawDetection;
import static limelight.networktables.LimelightUtils.extractArrayEntry;

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
    NetworkTableEntry rawDetections = camera.getNTTable().getEntry("rawdetections");

    public RealODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);
    }
    
    private void processDetections() {
        var rawDetectionArray = rawDetections.getDoubleArray(new double[0]);
        int valsPerEntry = 12;
        if (rawDetectionArray.length % valsPerEntry != 0) return;

        int numDetections = rawDetectionArray.length / valsPerEntry;

        for (int i = 0; i < numDetections; i++) {
            int baseIndex = i * valsPerEntry; // Starting index for this detection's data
            //   int    classId   = (int) extractArrayEntry(rawDetectionArray, baseIndex);
            double txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1);
            double tync = extractArrayEntry(rawDetectionArray, baseIndex + 2);
            //   double ta        = extractArrayEntry(rawDetectionArray, baseIndex + 3);
            //   double corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4);
            //   double corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5);
            //   double corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6);
            //   double corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7);
            //   double corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8);
            //   double corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9);
            //   double corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10);
            //   double corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11);

            Translation2d targetLocation = getRobotToObject(txnc, tync);
            double detectDist = targetLocation.getDistance(Translation2d.kZero);
            if (detectDist < ODVisionConstants.kMaxDetectRadiusMeters && detectDist > ODVisionConstants.kMinDetectRadiusMeters) {
                gamePieces.add(convertPieceToField(targetLocation, robotPose.get()));
            }
        }
    }
    
    @Override
    public void update() {
        gamePieces.clear();
        processDetections();
        // RawDetection[] results = camera.getData().getRawDetections();

        // for (var target : results) {
        //     Translation2d targetLocation = getRobotToObject(target.txnc, target.tync);
        //     double detectDist = targetLocation.getDistance(Translation2d.kZero);
        //     if (detectDist < ODVisionConstants.kMaxDetectRadiusMeters && detectDist > ODVisionConstants.kMinDetectRadiusMeters) {
        //         gamePieces.add(convertPieceToField(targetLocation, robotPose.get()));
        //     }
        // }

        updateTelemetry();
    }
}
