package frc.robot.subsystems.vision;

import java.util.List;

import limelight.networktables.PoseEstimate;

public interface Vision {
    public void periodic();
    public List<PoseEstimate> getAllEstimates();
}
