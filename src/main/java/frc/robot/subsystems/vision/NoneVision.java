package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import limelight.networktables.PoseEstimate;

public class NoneVision implements Vision {
    ArrayList<PoseEstimate> estimates = new ArrayList<PoseEstimate>();

    @Override
    public void periodic() {}

    @Override
    public List<PoseEstimate> getAllEstimates() {
        return estimates;
    }
    
}
