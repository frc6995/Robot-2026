package frc.robot.subsystems.vision.detection;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public class NoneODVision extends ObjectVision {
    public NoneODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);
    }

    @Override
    public void update() {
    }
}
