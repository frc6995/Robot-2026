package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.generated.ChoreoVars;
import frc.robot.util.AllianceFlipUtil;

public enum POI {
    INSTANCE;

        public final Pose2d bluePose;
    public final Pose2d redPose;

        private POI(Pose2d bluePose) {
        this.bluePose = bluePose;
        redPose = AllianceFlipUtil.flipPose(bluePose);
    }

public class POI {

    private static POI instance = null;

    public Pose2d L_Start = AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Start);
    public Pose2d R_Start = AllianceFlipUtil.flipPose(ChoreoVars.Poses.R_Start);

    public static Pose2d getL_Start() {
        AllianceFlipUtil.flipPose(ChoreoVars.Poses.L_Start);
        return L_Start;
    }

    public static POI getInstance() {
        if (instance == null) {
            instance = new POI();
        }
        return instance;
    
    }



    public final Pose2d bluePose;
    public final Pose2d redPose;

        private POI(Pose2d bluePose) {
        this.bluePose = bluePose;
        redPose = AllianceFlipUtil.flipPose(bluePose);
    }
}
