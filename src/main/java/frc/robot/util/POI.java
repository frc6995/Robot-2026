package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.ChoreoVars;

public class POI {
    // ============= POSES =============
    public static final Supplier<Pose2d> BALLL1= () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLL1);
    public static final Supplier<Pose2d> TRL1= () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TRL1);
    public static final Supplier<Pose2d> HELPL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELPL1);
    public static final Supplier<Pose2d> DEPOT = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.DEPOT);
    public static final Supplier<Pose2d> STA1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> CL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.CL1);
    public static final Supplier<Pose2d> BALLR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALLR1);
    public static final Supplier<Pose2d> TRR1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TRR1);
    public static final Supplier<Pose2d> HUB1 = () -> AllianceFlipUtil.flipPose(new Pose2d(4.625, 4.05, new Rotation2d()));
    public static final Supplier<Pose2d> LLNoHoodZone = () -> AllianceFlipUtil.flipPose(new Pose2d(3.55, .42, new Rotation2d()));
    public static final Supplier<Pose2d> URNoHoodZone = () -> AllianceFlipUtil.flipPose(new Pose2d(5.65, 7.64, new Rotation2d()));
    // ============= ROTATIONS =============
    public static final Supplier<Rotation2d> testEntry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-90)));
}
    