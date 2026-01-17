package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.ChoreoVars;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAlign;
import frc.robot.util.ChoreoVariables;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class POI {

    public final static Pose2d testStart = ChoreoVars.Poses.testStart;
    public final static Pose2d testEnd = ChoreoVars.Poses.testEnd;

    /**
     * DEPRICATED! Creates a Pose2d from choreo variables
     * Warning, this is not codegen
     * 
     * @param poseName (from Choreo)
     * @return a new {@code Pose2d} with the specified pose's coordinates and
     *         rotation
     */
    public static Pose2d createChoreoVariablesPose(String poseName) {
        Pose2d bluePose = new Pose2d(
                ChoreoVariables.getPose(poseName).getX(),
                ChoreoVariables.getPose(poseName).getY(),
                ChoreoVariables.getPose(poseName).getRotation());

        return AllianceFlipUtil.flipPose(bluePose);
    }

    /**
     * Creates a Pose2d from Choreo and flips it based on alliance
     * 
     * @param poseName
     * @return
     */
    public static Pose2d flipChorPose(Pose2d poseName) {
        return AllianceFlipUtil.flipPose(poseName);
    }

}
