package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Inches;
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
    }

    private Limelight camera = new Limelight(ODVisionConstants.kCameraID);
    private Timer timer = new Timer();

    private StructArrayPublisher<Pose2d> objectPoses;
    private StructPublisher<Pose2d> averageObjectPose;
    private StructPublisher<Pose2d> bestObjectPose;

    public RealODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);
        timer.start();

        var table = NetworkTableInstance.getDefault().getTable("Vision/Object-Detection/" + ODVisionConstants.kCameraID);
        objectPoses = table.getStructArrayTopic("Object Poses", Pose2d.struct).publish();
        averageObjectPose = table.getStructTopic("Average Pose", Pose2d.struct).publish();
        bestObjectPose = table.getStructTopic("Best Pose", Pose2d.struct).publish();
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

    private void updateTelemetry() {
        if(RobotContainer.kTelemetryVerbosity == TelemetryVerbosity.HIGH) {
            objectPoses.accept(getObjectPoses().toArray(new Pose2d[gamePieces.size()]));
        }
        if(RobotContainer.kTelemetryVerbosity.compareTo(TelemetryVerbosity.MID) >= 0) {
            var avgOpt = getAverageObjectLocation();
            averageObjectPose.accept(avgOpt.isPresent() ? new Pose2d(avgOpt.get(), Rotation2d.kZero) : Pose2d.kZero);

            var bestOpt = getBestObjectLocation();
            bestObjectPose.accept(bestOpt.isPresent() ? new Pose2d(bestOpt.get(), Rotation2d.kZero) : Pose2d.kZero);
        }
    }
}
