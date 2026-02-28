package frc.robot.subsystems.vision.detection;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NoneODVision extends ObjectVision {
    public NoneODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);

        var pose = robotPose.get();
        
        for(int i = 0; i < 15; i++) {
            Translation2d newPiece = new Translation2d(
                (Math.random() + 1.5) * 3.0,
                Math.random() * 3.0
            );

            gamePieces.add(
                new GamePiece(pose.getTranslation().plus(newPiece), timer.get())
            );
        }
    }

    @Override
    public void update() {
        // gamePieces.clear();
        updateTelemetry();
    }
}
