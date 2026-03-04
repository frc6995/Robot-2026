package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.random.RandomGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.AllianceFlipUtil;

public class SimODVision extends ObjectVision {

    public static final Translation2d[] m_fuel = new Translation2d[50];

    public static final Distance validRadius = Meters.of(2);

    public ArrayList<Translation2d> invalidGamepieces = new ArrayList<Translation2d>();

    public SimODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);

        for (int i = 0; i < m_fuel.length; i++) {
            m_fuel[i] = new Translation2d(
                    RandomGenerator.getDefault().nextDouble(AllianceFlipUtil.FIELD_LENGTH), 
                    RandomGenerator.getDefault().nextDouble(AllianceFlipUtil.FIELD_WIDTH));
        }
        
    }

    @Override
    public void update() {
        gamePieces.clear();
        invalidGamepieces.clear();
        for (Translation2d gamePiece : m_fuel) {
            if (isValid(gamePiece, robotPose.get().getTranslation())) {
                gamePieces.add(gamePiece);
            }
        }

        updateTelemetry();
    }

    public boolean isValid(Translation2d gamepiece, Translation2d robot) {
        return robot.getDistance(gamepiece) <= validRadius.in(Meters);
    }

}
