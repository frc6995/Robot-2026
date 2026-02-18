package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation3d;
import limelight.networktables.PoseEstimate;

public class NoneVision extends Vision {
    public NoneVision() {}
    public NoneVision(Supplier<Rotation3d> gyroRotation, Consumer<Rotation3d> resetRotation) {}
    
    @Override
    public void periodic() {}

    @Override
    public List<PoseEstimate> getAllEstimates() {
        return estimates;
    }
    
}
