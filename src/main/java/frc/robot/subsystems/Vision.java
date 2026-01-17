package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.BotPose;
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

    }
    private Limelight[] limelights;

    private Pigeon2 gyro;

    public Vision(Pigeon2 gyro) {
        this.gyro = gyro;
        
        limelights = new Limelight[VisionConstants.LL_IDS.length];

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new Limelight(VisionConstants.LL_IDS[i]);

            limelights[i].getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(VisionConstants.LL_OFFSETS[i])
                .save();
        }

        List<PoseEstimate> initialEstimates = getAllEstimates();

        Pose3d initialEstimate = (initialEstimates.size() != 0) ? initialEstimates.get(0).pose : new Pose3d(new Translation3d(), gyro.getRotation3d());

        for(Limelight limelight : limelights) {
            limelight.getSettings()
            .withRobotOrientation(new Orientation3d(
                initialEstimate.getRotation(), 
                new AngularVelocity3d(
                    gyro.getAngularVelocityXWorld().getValue(),
                    gyro.getAngularVelocityYWorld().getValue(),
                    gyro.getAngularVelocityZWorld().getValue())));
            gyro.setYaw(initialEstimate.getRotation().getMeasureZ());
        }
    }

    public List<PoseEstimate> getAllEstimates() {
        ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>(limelights.length);

        for(Limelight limelight : limelights) {
            var est = BotPose.BLUE_MEGATAG2.get(limelight);

            if(est.isPresent()) estimates.add(est.get());
        }

        return estimates;
    }
}