package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import limelight.Limelight;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RealODVision extends ObjectVision {
    public static class ODVisionConstants {
        public static final String kCameraID = "limelight-front";
        public static final Pose3d kCameraOffset = new Pose3d(
            new Translation3d(),
            new Rotation3d()
        );
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

        if(results.isPresent()) {
            var detectorTargets = results.get().targets_Detector;
            for(var target : detectorTargets) {
                Translation2d targetLocation = getRobotToObject(target.tx_nocrosshair, target.ty_nocrosshair);
                gamePieces.add(new GamePiece(robotPose.get().getTranslation().plus(targetLocation), timer.get()));
            }
        }

        updateTelemetry();
    }
    
}
