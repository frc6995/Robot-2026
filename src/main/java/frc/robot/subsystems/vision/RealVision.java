package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

public class RealVision extends Vision {
    public static class VisionConstants {
        public static final String[] LL_IDS = {
            "limelight-climb"
        };
        public static final Pose3d[] LL_OFFSETS = {
            new Pose3d( // climb
                new Translation3d(Inches.of(-10.925),Inches.of(10.750),Inches.of(8.820)),
                new Rotation3d(Degrees.of(7.52), Degrees.of(21.07), Degrees.of(180+20)))
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG2;

        // public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(0.5, 0.5, 999999);

    }
    private VisionModule[] limelights;

    private final Supplier<Rotation3d> gyroRotation;
    private final Consumer<Rotation3d> resetRotation;

    private final NetworkTable visionTable;

    private boolean headingSeeded = false;

    private final BooleanPublisher headingSeededPublisher;
    private final StructPublisher<Pose3d> seededPosePublisher;

    public RealVision(Supplier<Rotation3d> gyroRotation, Consumer<Rotation3d> resetRotation) {
        this.gyroRotation = gyroRotation;
        this.resetRotation = resetRotation;
        
        limelights = new VisionModule[VisionConstants.LL_IDS.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();
        seededPosePublisher = visionTable.getStructTopic("SeededPose", Pose3d.struct).publish();

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new VisionModule(VisionConstants.LL_IDS[i], VisionConstants.LL_OFFSETS[i], visionTable);
        }

        if(VisionConstants.kDefaultMode == EstimationMode.MEGATAG1) {
            headingSeeded = true;
            seededPosePublisher.accept(new Pose3d());
        }
    }

    private final AngularVelocity3d zeroAngularVelocity = new AngularVelocity3d(
                        DegreesPerSecond.zero(),
                        DegreesPerSecond.zero(),
                        DegreesPerSecond.zero());
    public void periodic() {
        if(!headingSeeded) {
            var initialEstimate = limelights[0].getPoseMT1();
            
            if(initialEstimate.isEmpty()|| initialEstimate.get().pose.getTranslation().getDistance(new Translation3d()) < 0.05) return;

            var initialPose = initialEstimate.get().pose;

            for(VisionModule limelight : limelights) {
                limelight.seedOrientation(new Orientation3d(
                    initialPose.getRotation(), 
                    zeroAngularVelocity));
            }

            resetRotation.accept(initialPose.getRotation());
            seededPosePublisher.accept(initialPose);
            headingSeeded = true;
        } else {
            for(VisionModule limelight : limelights) {
                limelight.periodic();
                
                limelight.seedOrientation(
                    new Orientation3d(
                        gyroRotation.get(),
                        zeroAngularVelocity
                    )
                );
            }
        }
        headingSeededPublisher.accept(headingSeeded);
    }

    public List<PoseEstimate> getAllEstimates() {
        ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>(0);

        if(!headingSeeded) return estimates;

        for(VisionModule limelight : limelights) {
            var est = limelight.getPose();

            if(est.isPresent()) {
                estimates.add(est.get());
            }
        }

        return estimates;
    }
}