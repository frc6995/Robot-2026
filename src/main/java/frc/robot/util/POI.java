package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.ChoreoVars;
import static frc.robot.util.AllianceFlipUtil.flipped;

public class POI {
    // ============= POSES =============

    // Center line intake start poses
    public static final Supplier<Pose2d> BALLL1 = flipped(ChoreoVars.Poses.BALLL1);
    public static final Supplier<Pose2d> BALLL2 = flipped(ChoreoVars.Poses.BALLL2);
    public static final Supplier<Pose2d> BALLL3 = flipped(ChoreoVars.Poses.BALLL3);

    public static final Supplier<Pose2d> BALLR1 = flipped(ChoreoVars.Poses.BALLR1);
    public static final Supplier<Pose2d> BALLR2 = flipped(ChoreoVars.Poses.BALLR2);
    public static final Supplier<Pose2d> BALLR3 = flipped(ChoreoVars.Poses.BALLR3);

    // Center line intake stop poses
    public static final Supplier<Pose2d> STOPL1 = flipped(ChoreoVars.Poses.STOPL1);
    public static final Supplier<Pose2d> STOPL2 = flipped(ChoreoVars.Poses.STOPL2);
    public static final Supplier<Pose2d> STOPL3 = flipped(ChoreoVars.Poses.STOPL3);

    public static final Supplier<Pose2d> STOPR1 = flipped(ChoreoVars.Poses.STOPR1);
    public static final Supplier<Pose2d> STOPR2 = flipped(ChoreoVars.Poses.STOPR2);
    public static final Supplier<Pose2d> STOPR3 = flipped(ChoreoVars.Poses.STOPR3);

    // Auto start poses
    public static final Supplier<Pose2d> TRL1 = flipped(ChoreoVars.Poses.TRL1);
    public static final Supplier<Pose2d> TRL2 = flipped(ChoreoVars.Poses.TRL2);

    public static final Supplier<Pose2d> TRR1 = flipped(ChoreoVars.Poses.TRR1);
    public static final Supplier<Pose2d> TRR2 = flipped(ChoreoVars.Poses.TRR2);

    // Center line help poses
    public static final Supplier<Pose2d> HELPL1 = flipped(ChoreoVars.Poses.HELPL1);
    public static final Supplier<Pose2d> HELPL2 = flipped(ChoreoVars.Poses.HELPL2);
    public static final Supplier<Pose2d> HELPL3 = flipped(ChoreoVars.Poses.HELPL3);

    public static final Supplier<Pose2d> HELPR1 = flipped(ChoreoVars.Poses.HELPR1);
    public static final Supplier<Pose2d> HELPR2 = flipped(ChoreoVars.Poses.HELPR2);
    public static final Supplier<Pose2d> HELPR3 = flipped(ChoreoVars.Poses.HELPR3);

    // Climb poses
    public static final Supplier<Pose2d> CL1 = flipped(ChoreoVars.Poses.CL1);
    public static final Supplier<Pose2d> CL2 = flipped(ChoreoVars.Poses.CL2);

    // Depot poses
    public static final Supplier<Pose2d> DEPOT_START = flipped(ChoreoVars.Poses.DEPOTSTART);
    public static final Supplier<Pose2d> DEPOT_END = flipped(ChoreoVars.Poses.DEPOTEND);
    public static final Supplier<Pose2d> DEPOT_HELP = flipped(ChoreoVars.Poses.HELPD1);

    // Other poses
    public static final Supplier<Pose2d> STA1 = flipped(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> HUB1 = flipped(new Pose2d(4.625, 4.05, new Rotation2d()));

    // ============= ROTATIONS =============

    // Center line intake pose rotations
    public static final Supplier<Rotation2d> BALLL2Entry = flipped(new Rotation2d(Degrees.of(-35)));
    public static final Supplier<Rotation2d> BALLL2CloseEntry = flipped(new Rotation2d(Degrees.of(0)));

    public static final Supplier<Rotation2d> BALLR2Entry = flipped(new Rotation2d(Degrees.of(35)));
    public static final Supplier<Rotation2d> BALLR2CloseEntry = flipped(new Rotation2d(Degrees.of(180)));

    // Auto start pose rotations
    public static final Supplier<Rotation2d> TRL1Entry = flipped(new Rotation2d(Degrees.of(180)));

    public static final Supplier<Rotation2d> TRR1Entry = flipped(new Rotation2d(Degrees.of(180)));

    // Autoalign help pose rotations
    public static final Supplier<Rotation2d> HELPL1Entry = flipped(new Rotation2d(Degrees.of(0)));
    public static final Supplier<Rotation2d> HELPL2Entry = flipped(new Rotation2d(Degrees.of(160)));
    public static final Supplier<Rotation2d> HELPL2CloseEntry = flipped(new Rotation2d(Degrees.of(140)));

    public static final Supplier<Rotation2d> HELPR1Entry = flipped(new Rotation2d(Degrees.of(-30)));
    public static final Supplier<Rotation2d> HELPR2Entry = flipped(new Rotation2d(Degrees.of(-160)));
    public static final Supplier<Rotation2d> HELPR2CloseEntry = flipped(new Rotation2d(Degrees.of(-140)));

    // Climb pose rotations
    // Other rotations
    public static final Supplier<Rotation2d> testEntry = flipped(new Rotation2d(Degrees.of(-90)));
    public static final Supplier<Rotation2d> depotStartEntry = flipped(new Rotation2d(Degrees.of(200)));

    // ============= TRANSLATIONS =============
    // ============= DISTANCES =============
    public static final Distance kOriginToTrenchBlue = Meters.of(4.6);
    public static final Distance kOriginToTrenchRed = Meters.of(11.9);
    
    // ============= PRIVATE RECTANGLES =============
        /** Number of meters over the center/side lines acceptable for the auto game piece pickup. */
    private static final double kLineToleranceMeters = 1.0;
    private static final Rectangle2d kLeftBlueBounds = new Rectangle2d(
        new Translation2d(
            0, AllianceFlipUtil.FIELD_WIDTH - kLineToleranceMeters),
        new Translation2d(
            AllianceFlipUtil.FIELD_LENGTH / 2.0 + kLineToleranceMeters, AllianceFlipUtil.FIELD_WIDTH / 2.0 - kLineToleranceMeters)
    );
    private static final Rectangle2d kRightBlueBounds = new Rectangle2d(
        new Translation2d(
            0, kLineToleranceMeters),
        new Translation2d(
            AllianceFlipUtil.FIELD_LENGTH / 2.0 + kLineToleranceMeters, AllianceFlipUtil.FIELD_WIDTH / 2.0 + kLineToleranceMeters
        )
    );
    private static final Rectangle2d kLeftRedBounds = new Rectangle2d(
        new Translation2d(
            AllianceFlipUtil.FIELD_LENGTH, 0),
        new Translation2d(
            AllianceFlipUtil.FIELD_LENGTH / 2.0 - kLineToleranceMeters, AllianceFlipUtil.FIELD_WIDTH / 2.0 + kLineToleranceMeters)
    );
    private static final Rectangle2d kRightRedBounds = new Rectangle2d(
        new Translation2d(
            AllianceFlipUtil.FIELD_LENGTH, AllianceFlipUtil.FIELD_WIDTH - kLineToleranceMeters),
        new Translation2d(
            AllianceFlipUtil.FIELD_LENGTH / 2.0 - kLineToleranceMeters, AllianceFlipUtil.FIELD_WIDTH / 2.0 - kLineToleranceMeters)
    );

    // ============= RECTANGLES =============
    public static final Supplier<Rectangle2d> kLeftAutoBounds = AllianceFlipUtil.constant(kLeftBlueBounds, kLeftRedBounds);
    public static final Supplier<Rectangle2d> kRightAutoBounds = AllianceFlipUtil.constant(kRightBlueBounds, kRightRedBounds);
}