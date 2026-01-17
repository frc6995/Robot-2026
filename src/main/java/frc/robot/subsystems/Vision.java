package frc.robot.subsystems;

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

public class Vision {
    public static class VisionConstants {
        public static final String[] LL_IDS = {
            "frontLL"
        };
        public static final Pose3d[] LL_OFFSETS = {
            new Pose3d( // frontLL
                new Translation3d(0,-1,19),
                new Rotation3d())
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG2;

    }
    private VisionModule[] limelights;

    private final Pigeon2 gyro;

    private final NetworkTable visionTable;

    public Vision(Pigeon2 gyro) {
        this.gyro = gyro;
        limelights = new VisionModule[VisionConstants.LL_IDS.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new VisionModule(VisionConstants.LL_IDS[i], VisionConstants.LL_OFFSETS[i], visionTable);
        }

        List<PoseEstimate> initialEstimates = getAllEstimates();

        Pose3d initialEstimate = (initialEstimates.size() != 0) ? initialEstimates.get(0).pose : new Pose3d(new Translation3d(), gyro.getRotation3d());

        for(VisionModule limelight : limelights) {
            limelight.seedOrientation(new Orientation3d(
                initialEstimate.getRotation(), 
                new AngularVelocity3d(
                    gyro.getAngularVelocityXWorld().getValue(),
                    gyro.getAngularVelocityYWorld().getValue(),
                    gyro.getAngularVelocityZWorld().getValue())));
            gyro.setYaw(initialEstimate.getRotation().getMeasureZ());
        }
    }

    public void periodic() {
        for(VisionModule limelight : limelights) {
            limelight.periodic();
        }
    }

    public List<PoseEstimate> getAllEstimates() {
        ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>(limelights.length);

        for(VisionModule limelight : limelights) {
            var est = limelight.getPose();

            if(est.isPresent()) estimates.add(est.get());
        }

        return estimates;
    }
}