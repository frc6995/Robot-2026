package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.ChoreoVars;

public class POI {
    // ============= POSES =============
    public static final Supplier<Pose2d> BALLL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLL1);
    public static final Supplier<Pose2d> TRL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TRL1);
    public static final Supplier<Pose2d> HELPL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPL1);
    public static final Supplier<Pose2d> DEPOT = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.DEPOT);
    public static final Supplier<Pose2d> STA1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> CL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.CL1);
    public static final Supplier<Pose2d> BALLR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLR1);
    public static final Supplier<Pose2d> TRR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TRR1);
    public static final Supplier<Pose2d> HELPR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPR1);
    public static final Supplier<Pose2d> HELPL2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPL2);
    public static final Supplier<Pose2d> BALLL2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLL2);
    public static final Supplier<Pose2d> HUB1 = () -> AllianceFlipUtil.flipPose(new Pose2d(4.625, 4.05, new Rotation2d()));
    // ============= ROTATIONS =============
    public static final Supplier<Rotation2d> testEntry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-90)));
    public static final Supplier<Rotation2d> TRL1Entry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(180)));
    public static final Supplier<Rotation2d> TRR1Entry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> HELPL1Entry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> HELPR1Entry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-30)));
    public static final Supplier<Rotation2d> HELPL2Entry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(160)));
    public static final Supplier<Rotation2d> BALLR1Entry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-75)));
    // ============= TRANSLATIONS =============
    // ============= DISTANCES =============
    private static final Distance kOriginToTrenchBlue = Meters.of(4.6);
    private static final Distance kOriginToTrenchRed = Meters.of(11.9);

    public static final Supplier<Distance> kOriginToTrench = () -> !AllianceFlipUtil.isRedAlliance() ? kOriginToTrenchBlue : kOriginToTrenchRed;
}
