package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTag;
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
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

public class RealATVision extends AprilTagVision {
    public static class ATVisionConstants {
        public static final String[] LL_IDS = {
            "limelight-climb", 
            "limelight-right", "limelight-left"
        };


        public static final Pose3d[] LL_OFFSETS = {
            new Pose3d( // climb
                new Translation3d(Inches.of(-11.0672),Inches.of(-10.432), Inches.of(8.674)),
                new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(180))),
            new Pose3d( // right
                new Translation3d(Inches.of(2.550), Inches.of(12.987),Inches.of(7.435)),
                new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(-90))),
            new Pose3d( // left
                new Translation3d(Inches.of(2.550), Inches.of(-12.987), Inches.of(7.435)),
                new Rotation3d(Degrees.zero(), Degrees.of(22.5), Degrees.of(90)))
        };
        public static final EstimationMode kDefaultMode = EstimationMode.MEGATAG2;

        public static final Matrix<N3, N1> KNormalStdDevs = VecBuilder.fill(0.175, 0.175, 999999);
        public static final Matrix<N3, N1> kDisabledStdDevs = VecBuilder.fill(0.0001, 0.0001, 0.0001);

    }
    
    private final AngularVelocity3d zeroAngularVelocity = new AngularVelocity3d(
                        DegreesPerSecond.zero(),
                        DegreesPerSecond.zero(),
                        DegreesPerSecond.zero());

    private AprilTagModule[] limelights;

    private final Supplier<Rotation3d> gyroRotation;
    private final Consumer<Pose2d> resetPose;

    private final NetworkTable visionTable;

    private boolean headingSeeded = false;

    private final BooleanPublisher headingSeededPublisher;
    private final StructPublisher<Pose3d> seededPosePublisher;

    public RealATVision(Supplier<Rotation3d> gyroRotation, Consumer<Pose2d> resetPose) {
        this.gyroRotation = gyroRotation;
        this.resetPose = resetPose;

        limelights = new AprilTagModule[ATVisionConstants.LL_IDS.length];

        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        headingSeededPublisher = visionTable.getBooleanTopic("HeadingSeeded").publish();
        seededPosePublisher = visionTable.getStructTopic("SeededPose", Pose3d.struct).publish();

        for(int i = 0; i < limelights.length; i++) {
            limelights[i] = new AprilTagModule(ATVisionConstants.LL_IDS[i], ATVisionConstants.LL_OFFSETS[i], visionTable);
        }
    }

    public void periodic() {
        estimates.clear();
        if(DriverStation.isDisabled() || !headingSeeded) {
            for(AprilTagModule limelight : limelights) {
                limelight.periodic();
                var result = limelight.getPoseMT1();
                if(result.isPresent() && result.get().pose.getTranslation().getDistance(Translation3d.kZero) > 0.05) {
                    estimates.add(result.get());
                    headingSeeded = true;
                }
            }
            seededPosePublisher.accept(new Pose3d(Translation3d.kZero, gyroRotation.get()));
        } else {
            if(!headingSeeded) headingSeeded = true;
            Orientation3d newOrientation = new Orientation3d(
                        gyroRotation.get(),
                        zeroAngularVelocity);
                        
            for(AprilTagModule limelight : limelights) {
                limelight.periodic();
                
                limelight.seedOrientation(
                    newOrientation
                );
                var estSupp = limelight.getPose();

                if(estSupp.isPresent()) {
                    estimates.add(estSupp.get());
                }
            }
        }
        headingSeededPublisher.accept(headingSeeded);
    }

    @Override
    public List<PoseEstimate> getAllEstimates() {
        return estimates;
    }
}