package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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

public class RealATVision extends AprilTagVision {
    public static class ATVisionConstants {
        public static final String[] kCameraIDs = {
            "limelight-climb"
        };
        public static final Pose3d[] kCameraOffsets = {
            new Pose3d( // climb
                new Translation3d(Inches.of(-10.925),Inches.of(10.750),Inches.of(8.820)),
                new Rotation3d(Degrees.of(7.52), Degrees.of(20), Degrees.of(180+22.5)))
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG2;

        public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(0.00001, 0.000001, 0.00001);

    }
    private static final AngularVelocity3d zeroAngularVelocity3d = new AngularVelocity3d(
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0));

    private AprilTagModule[] limelights;

    private final Supplier<Rotation3d> gyroRotation;
    private final Consumer<Rotation3d> resetRotation;

    private final NetworkTable visionTable;

    private boolean headingSeeded = false;

    // private final BooleanPublisher headingSeededPublisher;
    // private final StructPublisher<Pose3d> seededPosePublisher;

    public RealATVision(Supplier<Rotation3d> gyroRotation, Consumer<Rotation3d> resetRotation) {
        this.gyroRotation = gyroRotation;
        this.resetRotation = resetRotation;
        
        limelights = new AprilTagModule[ATVisionConstants.kCameraIDs.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision/AprilTag");
        // headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();
        // seededPosePublisher = visionTable.getStructTopic("SeededPose", Pose3d.struct).publish();

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new AprilTagModule(ATVisionConstants.kCameraIDs[i], ATVisionConstants.kCameraOffsets[i], visionTable);
        }

        if(ATVisionConstants.kDefaultMode == EstimationMode.MEGATAG1) {
            headingSeeded = true;
            // seededPosePublisher.accept(Pose3d.kZero);
        }
    }

    private final AngularVelocity3d zeroAngularVelocity = new AngularVelocity3d(
                        DegreesPerSecond.zero(),
                        DegreesPerSecond.zero(),
                        DegreesPerSecond.zero());
    public void periodic() {
        estimates.clear();
        if(!headingSeeded) {
            // var initialEstimate = limelights[0].getPoseMT1();
            
            // if(initialEstimate.isEmpty()|| initialEstimate.get().pose.getTranslation().getDistance(new Translation3d()) < 0.05) return;

            // var initialPose = initialEstimate.get().pose;

            // for(VisionModule limelight : limelights) {
            //     limelight.seedOrientation(new Orientation3d(
            //         initialPose.getRotation(), 
            //         zeroAngularVelocity));
            // }

            // resetRotation.accept(initialPose.getRotation());
            // seededPosePublisher.accept(initialPose);
            headingSeeded = true;
        } else {
            for(AprilTagModule limelight : limelights) {
                limelight.periodic();
                
                limelight.seedOrientation(
                    new Orientation3d(
                        gyroRotation.get(),
                        zeroAngularVelocity
                    )
                );
                var estSupp = limelight.getPose();

                if(estSupp.isPresent()) {
                    estimates.add(estSupp.get());
                }
            }
        }
        // headingSeededPublisher.accept(headingSeeded);
    }

    @Override
    public List<PoseEstimate> getAllEstimates() {
        return estimates;
    }
}