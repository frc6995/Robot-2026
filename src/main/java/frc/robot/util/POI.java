package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.ChoreoVars;

public class POI {
        // ============= POSES =============

        // Center line intake poses
        public static final Supplier<Pose2d> BALLL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLL1);
        public static final Supplier<Pose2d> BALLR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLR1);
        public static final Supplier<Pose2d> BALLL2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLL2);
        public static final Supplier<Pose2d> BALLL3 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLL3);

        // Auto start poses
        public static final Supplier<Pose2d> TRL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TRL1);
        public static final Supplier<Pose2d> TRR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TRR1);

        // Autoalign help poses
        public static final Supplier<Pose2d> HELPL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPL1);
        public static final Supplier<Pose2d> HELPR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPR1);
        public static final Supplier<Pose2d> HELPL2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPL2);

        // Climb poses
        public static final Supplier<Pose2d> CL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.CL1);

        // Other poses
        public static final Supplier<Pose2d> DEPOT = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.DEPOT);
        public static final Supplier<Pose2d> STA1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.STA1);
            public static final Supplier<Pose2d> HUB1 = () -> AllianceFlipUtil.flipPose(new Pose2d(4.625, 4.05, new Rotation2d()));

        // ============= ROTATIONS =============

        // Center line intake pose rotations
        public static final Supplier<Rotation2d> BALLL2Entry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(-75)));

        // Auto start pose rotations
        public static final Supplier<Rotation2d> TRL1Entry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(180)));
        public static final Supplier<Rotation2d> TRR1Entry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(0)));

        // Autoalign help pose rotations
        public static final Supplier<Rotation2d> HELPL1Entry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(0)));
        public static final Supplier<Rotation2d> HELPR1Entry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(-30)));
        public static final Supplier<Rotation2d> HELPL2Entry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(160)));
        // Climb pose rotations
        // Other rotations
        public static final Supplier<Rotation2d> testEntry = () -> AllianceFlipUtil
                        .flipRotation(new Rotation2d(Degrees.of(-90)));

                            // ============= TRANSLATIONS =============
    // ============= DISTANCES =============
    private static final Distance kOriginToTrenchBlue = Meters.of(4.6);
    private static final Distance kOriginToTrenchRed = Meters.of(11.9);

    public static final Supplier<Distance> kOriginToTrench = () -> !AllianceFlipUtil.isRedAlliance() ? kOriginToTrenchBlue : kOriginToTrenchRed;
}