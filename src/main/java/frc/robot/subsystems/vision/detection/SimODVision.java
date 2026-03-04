package frc.robot.subsystems.vision.detection;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.function.Supplier;
import java.util.random.RandomGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;

import frc.robot.util.AllianceFlipUtil;
import frc.robot.subsystems.vision.detection.RealODVision.ODVisionConstants;;

public class SimODVision extends ObjectVision {

    public static final Pose3d[] m_fuel = new Pose3d[200];

    private final StructArrayPublisher<Pose3d> allFuelPoses;

    public SimODVision(Supplier<Pose2d> robotPose) {
        super(robotPose);

        for (int i = 0; i < m_fuel.length; i++) {
            double originX = AllianceFlipUtil.FIELD_LENGTH / 2.0;
            double originY = AllianceFlipUtil.FIELD_WIDTH / 2.0;
            m_fuel[i] = new Pose3d(new Translation3d(
                    RandomGenerator.getDefault().nextDouble(originX - 1.8, originX + 1.8), 
                    RandomGenerator.getDefault().nextDouble(originY - 2.7, originY + 2.7),
                    0),
                    Rotation3d.kZero);
        }
        var table = NetworkTableInstance.getDefault().getTable("Vision/Object-Detection/" + ODVisionConstants.kCameraID);
        allFuelPoses = table.getStructArrayTopic("AllFuelPoses",Pose3d.struct).publish();
        allFuelPoses.accept(m_fuel);
    }

    @Override
    public void update() {
        gamePieces.clear();
        for (Pose3d gamePiece : m_fuel) {
            Translation2d pieceTranslation = gamePiece.getTranslation().toTranslation2d();
            if (isValid(pieceTranslation, robotPose.get())) {
                gamePieces.add(pieceTranslation);
            }
        }

        updateTelemetry();
    }

    @Override
    public void updateTelemetry() {
        super.updateTelemetry();
    }

    public boolean isValid(Translation2d gamepiece, Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        double robotAngle = robotPose.getRotation().plus(Rotation2d.k180deg).getDegrees();
        double angle = robotTranslation.minus(gamepiece).getAngle().getDegrees();
        double detectDist = robotTranslation.getDistance(gamepiece);
        return  detectDist < ODVisionConstants.kMaxDetectRadiusMeters && detectDist > ODVisionConstants.kMinDetectRadiusMeters && MathUtil.isNear(robotAngle, angle, ODVisionConstants.kCameraFOVDegrees / 2.0);
    }

}
