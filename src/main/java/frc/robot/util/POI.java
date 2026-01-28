package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.ChoreoVars;

public class POI {
    // ============= POSES =============
    public static final Supplier<Pose2d> BALL1= () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALL1);
    public static final Supplier<Pose2d> TR1= () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TR1);
    public static final Supplier<Pose2d> HELP1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.HELP1);
    public static final Supplier<Pose2d> DEPOT = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.DEPOT);
    public static final Supplier<Pose2d> STA1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.STA1);
    public static final Supplier<Pose2d> CL1 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.CL1);
    public static final Supplier<Pose2d> BALL2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.BALL2);
    public static final Supplier<Pose2d> TR2 = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.TR2);
    // ============= ROTATIONS =============
    public static final Supplier<Rotation2d> testEntry = () -> AllianceFlipUtil
            .flipRotation(new Rotation2d(Degrees.of(-90)));
}
    