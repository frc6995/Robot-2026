package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APTarget;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.ChoreoVars;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakePivotS.intakeConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoAlign;
import frc.robot.util.ChoreoVariables;

public class Autos {

    public class AutoConstants {
        public static double DEFAULT_ACCELERATION = 15;
        public static double DEFAULT_JERK = 12;
    }

    private final AutoFactory m_factory;
    private final RobotContainer m_container;
    protected final CommandSwerveDrivetrain m_drivebase;
    protected final IntakePivotS m_intakepiv;
    private final double SCORE_WAIT = 0.875;

    public Autos(CommandSwerveDrivetrain drivebase, IntakePivotS intakepiv,
            AutoFactory factory, RobotContainer container) {
        m_drivebase = drivebase; // need
        m_intakepiv = intakepiv;
        m_factory = factory;
        m_container = container;

        container.m_chooser.addRoutine(choreoAutoName, this::choreoAuto);
        container.m_chooser.addRoutine(APName, this::APAuto);
    }

    Pose2d testStart = flipChorPose(ChoreoVars.Poses.testStart);
    Pose2d testEndPose = flipChorPose(ChoreoVars.Poses.testEnd);


    // Example auto
    String choreoAutoName = "Choreo Auto";

    public AutoRoutine choreoAuto() {
        final AutoRoutine routine = m_factory.newRoutine(choreoAutoName);
        final AutoTrajectory traj = routine.trajectory("OP");
        routine.active().onTrue(
                traj.resetOdometry()
                .andThen(traj.cmd())
                      );
        return routine;
    }

     // Example auto
    String APName = "AP Auto";
    public AutoRoutine APAuto() {
        final AutoRoutine routine = m_factory.newRoutine(APName);
        final AutoTrajectory odometry = routine.trajectory("resetOdometryStart");

        routine.active().onTrue(
                odometry.resetOdometry()
                        .andThen(defaultAlignRequest(testEndPose)));
        return routine;
    }

    public enum RobotState {
        // Todo: add all states as in button mapping doc
        CORAL_INTAKING,
        HANDOFF,
        L1_PRE_SCORE,
        L2_PRE_SCORE,
        L3_PRE_SCORE,
        L4_PRE_SCORE,
        INTAKING_ALGAE_GROUND,
        INTAKING_ALGAE_REEF,
        ALGAE_STOW,
        BARGE_PREP

    }

    public RobotState currentState = RobotState.HANDOFF;

    // Functions below:
    // Todo: add command that combines intakeCoral and stowCoral, update states

    public Command stowCoral() {
        return Commands.sequence(setState(RobotState.HANDOFF),
                m_intakepiv.setAngle(intakeConstants.ALGAE_INTAKE));
    }

    // Commands below:
    // TODO: add handoff sequence

    public Command prepL1() {
        return Commands.sequence(setState(RobotState.L1_PRE_SCORE),
                m_intakepiv.setAngle(intakeConstants.STOW)

        );
    }

    public Command setState(RobotState newState) {
        return Commands.runOnce(() -> {
            currentState = newState;
            System.out.println("State changed to: " + newState);
        });
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the
     * targetPose.
     * 
     * @param targetPose The desired ending Pose2d
     * @return The Command to navigate to the given Pose2d.
     */
    public Command defaultAlignRequest(Pose2d targetPose) {
        return new AutoAlign(new APTarget(AllianceFlipUtil.flipPose(targetPose)), m_drivebase,
                new APConstraints(AutoConstants.DEFAULT_ACCELERATION, AutoConstants.DEFAULT_JERK));
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the
     * targetPose. Takes a desired entry angle
     * when approaching the targetPose.
     * 
     * @param targetPose The desired ending Pose2d
     * @param entryAngle The desired angle to approach the targetPose with.
     * @return The Command to navigate to the given Pose2d.
     */
    public Command defaultAlignRequest(Pose2d targetPose, Rotation2d entryAngle) {
        return new AutoAlign(new APTarget(AllianceFlipUtil.flipPose(targetPose))
                .withEntryAngle(AllianceFlipUtil.flipRotation(entryAngle)), m_drivebase,
                new APConstraints(AutoConstants.DEFAULT_ACCELERATION, AutoConstants.DEFAULT_JERK));
    }

    /**
     * Creates a Pose2d from choreo variables
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
     * 
     * @param poseName
     * @return
    */
    public static Pose2d flipChorPose(Pose2d poseName) {
        return AllianceFlipUtil.flipPose(poseName);
    }
}
