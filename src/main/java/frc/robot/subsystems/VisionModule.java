package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.Vision.VisionConstants;
import limelight.Limelight;
import limelight.networktables.Orientation3d;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;

/**
 * Wrapper class for a Yet Another Limelight Library {@link limelight.Limelight} object. 
 * Records vision data to NetworkTables for debugging. 
 */
public class VisionModule {
    private final Limelight limelight;

    private final NetworkTable moduleSubTable;

    private final StructPublisher<Pose3d> estimatePublisher;
    private final BooleanPublisher isActivePublisher;
    private final StringPublisher modePublisher;
    private final StringPublisher defaultModePublisher;

    private EstimationMode defaultMode;
    private EstimationMode lastMode;

    public VisionModule(String limelightID, Pose3d offset, NetworkTable visionTable) {
        this.limelight = new Limelight(limelightID);
        limelight.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(offset)
            .save();

        defaultMode = VisionConstants.kDefaultMode;

            // Publishers for Limelight data
        moduleSubTable = visionTable.getSubTable(limelightID);
        estimatePublisher = moduleSubTable.getStructTopic("PoseEstimate", Pose3d.struct).publish();
        isActivePublisher = moduleSubTable.getBooleanTopic("IsActive").publish();
        modePublisher = moduleSubTable.getStringTopic("LastEstimateMode").publish();
        defaultModePublisher = moduleSubTable.getStringTopic("DefaultEstimateMode").publish();

        defaultModePublisher.setDefault(defaultMode.name());
    }

    /**
     * Must be called periodically in {@link frc.robot.subsystems.Vision#periodic()}
     */
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Updates the {@link edu.wpi.first.networktables.NetworkTable} subtable for the Limelight.
     * Records the latest pose estimate, whether or not the Limelight has estimate data, the current
     * {@link limelight.networktables.LimelightPoseEstimator.EstimationMode} for the robot, and the default
     * {@link limelight.networktables.LimelightPoseEstimator.EstimationMode}.
     */
    private void updateTelemetry() {
        estimatePublisher.accept(getPose().isPresent() ? getPose().get().pose : new Pose3d());
        isActivePublisher.accept(isActive());
        modePublisher.accept(lastMode.toString());
        defaultModePublisher.accept(defaultMode.toString());
    }

    /**
     * Checks if there is an AprilTag pose estimation.
     * 
     * @return Whether or not the Limelight is estimating a robot pose.
     */
    public boolean isActive() {
        return getPoseMT1().isPresent() && getPoseMT1().get().hasData;
    }

    /**
     * Retrieves the pose of the robot. Automatically swaps between MegaTag1 and MegaTag2 depending on the  
     * {@link VisionModule#defaultMode}. Returns {@link java.util.Optional#empty()}
     * if there are no results.
     * 
     * @return The estimated pose if the Limelight has targets
     */
    public Optional<PoseEstimate> getPose() {
        if(!isActive()) return Optional.empty();

        switch(defaultMode) {
            case MEGATAG1:
                return getPoseMT1();
            default:
                return getPoseMT2();
        }
    }

    /**
     * Retrieves the pose of the robot using MegaTag2. Must be seeded with an initial orientation before
     * use.
     * 
     * @return The estimated pose if the Limelight has targets and the initial orientation is seeded.
     */
    public Optional<PoseEstimate> getPoseMT2() {
        lastMode = EstimationMode.MEGATAG2;
        return BotPose.BLUE_MEGATAG2.get(limelight);
    }

    /**
     * Retrieves the pose of the robot using MegaTag1.
     * 
     * @return The estimated pose if the Limelight has targets
     */
    public Optional<PoseEstimate> getPoseMT1() {
        lastMode = EstimationMode.MEGATAG1;
        return BotPose.BLUE.get(limelight);
    }

    /**
     * Seeds the initial orientation of the Limelight for MegaTag2.
     * 
     * @param orientation3d The rotation and angular velocity of the robot.
     */
    public void seedOrientation(Orientation3d orientation3d) {
        limelight.getSettings().withRobotOrientation(orientation3d);
    }
}
