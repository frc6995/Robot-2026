package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;

public class RealVision extends Vision {
    public static class VisionConstants {
        public static final String[] LL_IDS = {
            "limelight-climb"
        };
        public static final Pose3d[] LL_OFFSETS = {
            new Pose3d( // frontLL
                new Translation3d(-0.0254,-0.0254,0.4826),
                new Rotation3d( -7.52, 21.07, 20))
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG2;

    }
    private VisionModule[] limelights;

    private final Pigeon2 gyro;

    private final NetworkTable visionTable;

    private boolean headingSeeded = false;

    private final BooleanPublisher headingSeededPublisher;

    public RealVision(Pigeon2 gyro) {
        this.gyro = gyro;
        limelights = new VisionModule[VisionConstants.LL_IDS.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new VisionModule(VisionConstants.LL_IDS[i], VisionConstants.LL_OFFSETS[i], visionTable);
        }
    }

    public void periodic() {
        headingSeededPublisher.accept(headingSeeded);

        if(!headingSeeded) {
            var initialEstimate = limelights[0].getPoseMT1();
            
            if(initialEstimate.isEmpty()) return;

            var initialPose = initialEstimate.get().pose;

            for(VisionModule limelight : limelights) {
                limelight.seedOrientation(new Orientation3d(
                    initialPose.getRotation(), 
                    new AngularVelocity3d(
                        gyro.getAngularVelocityXWorld().getValue(),
                        gyro.getAngularVelocityYWorld().getValue(),
                        gyro.getAngularVelocityZWorld().getValue())));
                gyro.setYaw(initialPose.getRotation().getMeasureZ());
            }

            headingSeeded = true;
        } else {
            for(VisionModule limelight : limelights) {
                limelight.periodic();
            }
        }
    }

    public List<PoseEstimate> getAllEstimates() {
        ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>(0);

        for(VisionModule limelight : limelights) {
            var est = limelight.getPose();

            if(est.isPresent()) {
                estimates.add(est.get());
            }
        }

        return estimates;
    }
}