package frc.robot.subsystems.vision;

import java.util.List;

import limelight.networktables.PoseEstimate;

public abstract class Vision {
    public abstract void periodic();
    public abstract List<PoseEstimate> getAllEstimates();
}
