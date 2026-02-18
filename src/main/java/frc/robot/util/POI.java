package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.generated.ChoreoVars;
import static frc.robot.util.AllianceFlipUtil.flipped;

public class POI {
    // ============= POSES =============

    // Center line intake poses
    public static final Supplier<Pose2d> BALLL1 = flipped(ChoreoVars.Poses.BALLL1);
    public static final Supplier<Pose2d> BALLR1 = flipped(ChoreoVars.Poses.BALLR1);
    public static final Supplier<Pose2d> BALLL2 = flipped(ChoreoVars.Poses.BALLL2);
    public static final Supplier<Pose2d> BALLL3 = flipped(ChoreoVars.Poses.BALLL3);

    // Auto start poses
    public static final Supplier<Pose2d> TRL1 = flipped(ChoreoVars.Poses.TRL1);
    public static final Supplier<Pose2d> TRR1 = flipped(ChoreoVars.Poses.TRR1);

    // Autoalign help poses
    public static final Supplier<Pose2d> HELPL1 = flipped(ChoreoVars.Poses.HELPL1);
    public static final Supplier<Pose2d> HELPR1 = flipped(ChoreoVars.Poses.HELPR1);
    public static final Supplier<Pose2d> HELPL2 = flipped(ChoreoVars.Poses.HELPL2);

    // Climb poses
    public static final Supplier<Pose2d> CL1 = flipped(ChoreoVars.Poses.CL1);

    // Other poses
    public static final Supplier<Pose2d> DEPOT = flipped(ChoreoVars.Poses.DEPOT);
    public static final Supplier<Pose2d> STA1 = flipped(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> HUB1 = flipped(new Pose2d(4.625, 4.05, new Rotation2d()));

    // ============= ROTATIONS =============

    // Center line intake pose rotations
    public static final Supplier<Rotation2d> BALLL2Entry = flipped(new Rotation2d(Degrees.of(-75)));

    // Auto start pose rotations
    public static final Supplier<Rotation2d> TRL1Entry = flipped(new Rotation2d(Degrees.of(180)));
    public static final Supplier<Rotation2d> TRR1Entry = flipped(new Rotation2d(Degrees.of(0)));

    // Autoalign help pose rotations
    public static final Supplier<Rotation2d> HELPL1Entry = flipped(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> HELPR1Entry = flipped(new Rotation2d(Degrees.of(-30)));
    public static final Supplier<Rotation2d> HELPL2Entry = flipped(new Rotation2d(Degrees.of(160)));
    // Climb pose rotations
    // Other rotations
    public static final Supplier<Rotation2d> testEntry = flipped(new Rotation2d(Degrees.of(-90)));

    // ============= TRANSLATIONS =============
    // ============= DISTANCES =============
    public static final Distance kOriginToTrenchBlue = Meters.of(4.6);
    public static final Distance kOriginToTrenchRed = Meters.of(11.9);
}