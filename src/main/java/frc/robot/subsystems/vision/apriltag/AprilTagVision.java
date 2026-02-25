package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.List;

import limelight.networktables.PoseEstimate;

public abstract class AprilTagVision {
    protected ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>(0);

    public abstract void periodic();
    public abstract List<PoseEstimate> getAllEstimates();
}
