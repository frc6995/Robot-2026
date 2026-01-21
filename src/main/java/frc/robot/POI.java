package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.ChoreoVars;
import frc.robot.util.AllianceFlipUtil;

public class POI {

    public static final Supplier<Pose2d> L_Start = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Start);
    public static final Supplier<Pose2d> R_Start = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Start);
    public static final Supplier<Pose2d> L_Sweep = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Sweep);
    public static final Supplier<Pose2d> R_Sweep = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Sweep);
    public static final Supplier<Pose2d> C_ClimbPose = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.C_ClimbPose);
    public static final Supplier<Pose2d> R_ClimbPose = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Climb_Pose);
    public static final Supplier<Pose2d> L_ClimbPose = () -> AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_ClimbPose);
}