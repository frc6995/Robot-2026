package frc.robot.autos;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.therekrab.autopilot.APProfile;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer.RobotStates;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climb.climbextension.ClimbExtensionS;
import frc.robot.subsystems.flywheel.FlyWheelS;
import frc.robot.subsystems.hood.HoodS;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
import frc.robot.subsystems.indexer.IndexerS;
import frc.robot.subsystems.indexer.RealIndexerS.IndexerConstants;
import frc.robot.subsystems.intakepivot.IntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS.IntakePivotConstants;
import frc.robot.subsystems.intakeroller.IntakeRollerS;
import frc.robot.subsystems.intakeroller.RealIntakeRollerS.IntakeRollerConstants;
import frc.robot.subsystems.spindexer.SpindexerS;
import frc.robot.subsystems.turret.TurretS;
import frc.robot.subsystems.vision.detection.ObjectVision;
import frc.robot.subsystems.vision.detection.ObjectVision.Cluster;
import frc.robot.subsystems.spindexer.RealSpindexerS.SpindexerConstants;
import frc.robot.util.AutoAlign;
import frc.robot.util.DriveUtil;
import frc.robot.util.POI;
import frc.robot.util.TriggerUtil;

public class AutoCommands {
    // You need these dependencies passed in
    private final CommandSwerveDrivetrain m_drivebase;
    private final HoodS m_hood;
    private final IntakePivotS m_intakePivot;
    private final IntakeRollerS m_intakeRoller;
    private final TurretS m_turret;
    private final IndexerS m_indexer;
    private final SpindexerS m_Spindexer;
    private final FlyWheelS m_flywheel;
    private final ClimbExtensionS m_climbExtension;
    private final ObjectVision m_objectVision;
    private final RobotStates m_robotStates;

    SwerveRequest m_intakeDriveRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSpeeds(new ChassisSpeeds(1.6, 0.0, 0));

        final BooleanSupplier isJammed;
        final Supplier<Voltage> runSpindexerWithReverse;

    public AutoCommands(
            CommandSwerveDrivetrain drivebase,
            HoodS hood, IntakePivotS intakePivot,
            IntakeRollerS intakeRoller, TurretS turret,
            IndexerS indexer, SpindexerS spindexer,
            FlyWheelS flyWheel, ClimbExtensionS climbExtension, ObjectVision objectVision, RobotStates robotStates) {
        this.m_drivebase = drivebase;
        this.m_hood = hood;
        this.m_intakePivot = intakePivot;
        this.m_intakeRoller = intakeRoller;
        this.m_turret = turret;
        this.m_indexer = indexer;
        this.m_Spindexer = spindexer;
        this.m_flywheel = flyWheel;
        this.m_climbExtension = climbExtension;
        this.m_objectVision = objectVision;
        this.m_robotStates = robotStates;

        isJammed = new Trigger(() ->  m_Spindexer.getCurrent().gt(kUnjamThreshold)).debounce(1, DebounceType.kRising).debounce(0.25, DebounceType.kFalling);
        runSpindexerWithReverse = () -> {
                return isJammed.getAsBoolean() ? SpindexerConstants.kReverseVoltage : SpindexerConstants.kFastVoltage;
        };
    }

    // Create a trigger that watches your condition
    /**
     * Creates a routine that intakes from the center line
     * 
     * @param helpPose             Pose to go through on the way to the final pose
     *                             to intake
     * @param helpPoseEntryAngle
     * @param helpPoseTolerance    Minimum radius for robot to be in from helpPose
     *                             that triggers the next command
     * @param intakePose           Pose to go through before slowDriveForward
     * @param intakePoseEntryAngle
     * @param intakePoseTolerance  Mnimum radius for robot to be in from intakePose
     *                             that triggers the next command
     * @param stopPose             Pose to drive slowly towards while intaking
     * @return command that intakes from the center line
     */
    public Command APToIntake(Pose2d helpPose, Distance helpPoseTolerance, Pose2d intakePose,
            Rotation2d intakePoseEntryAngle, Distance intakePoseTolerance, Pose2d stopPose) {
        return Commands.deadline(
                Commands.sequence(
                        new AutoAlign(helpPose, m_drivebase,
                                AutoAlign.kDefaultVelocityLimitedProfile).until(
                                        TriggerUtil.isWithinRadius(
                                                () -> helpPose.getTranslation(),
                                                () -> m_drivebase.state.Pose,
                                                () -> helpPoseTolerance)),
                        new AutoAlign(intakePose, intakePoseEntryAngle, m_drivebase,
                                AutoAlign.kDefaultVelocityLimitedProfile).until(
                                        TriggerUtil.isWithinRadius(
                                                () -> intakePose.getTranslation(),
                                                () -> m_drivebase.state.Pose,
                                                () -> intakePoseTolerance)),
                        new AutoAlign(stopPose, m_drivebase, AutoAlign.kSlowDriveProfile)),
                Commands.parallel(fuelIntake(), m_hood.setAngle(() -> HoodConstants.kLowerLimit)));
    }

    /**
     * Creates a routine that intakes at the depot
     * 
     * @param helpPose           Pose to go through on the way to the final pose
     *                           to intake
     * @param helpPoseEntryAngle
     */

    public Command APtoDepot() {
        return Commands.deadline(
                Commands.sequence(
                        new AutoAlign(POI.DEPOT_START.get(), POI.depotStartEntry.get(),
                                m_drivebase,
                                AutoAlign.kSlowDriveProfile).until(
                                        TriggerUtil.isWithinRadius(
                                                () -> POI.DEPOT_START
                                                        .get()
                                                        .getTranslation(),
                                                () -> m_drivebase.state.Pose,
                                                () -> Meters.of(0.1))),
                        new AutoAlign(POI.DEPOT_END.get(), m_drivebase,
                                AutoAlign.kSlowDriveProfile)),
                Commands.parallel(fuelIntake()));
    }

    /**
     * Creates a command that intakes from the center line
     * 
     * @param helpPose             Pose to go through on the way to the final pose
     *                             for scoring
     * @param helpPoseEntryAngle
     * @param helpPoseTolerance    Minimum radius for robot to be in from helpPose
     *                             that triggers the next command
     * @param targetpose           Final pose to drive to for scoring
     * @param targetPoseEntryAngle
     * @return command that intakes from the center line
     */
    public Command APBackFromIntake(Pose2d helpPose,
            Rotation2d helpPoseEntryAngle,
            Distance helpPoseTolerance,
            Pose2d targetpose,
            Rotation2d targetPoseEntryAngle) {
        return Commands.sequence(
                new AutoAlign(helpPose, m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile).until(
                                TriggerUtil.isWithinRadius(
                                        () -> helpPose.getTranslation(),
                                        () -> m_drivebase.state.Pose,
                                        () -> helpPoseTolerance)),
                new AutoAlign(targetpose, targetPoseEntryAngle, m_drivebase,
                        AutoAlign.kDefaultVelocityLimitedProfile));

    }

    /**
     * Command that aligns to target pose and climbs
     * 
     * @param targetpose Target pose to autoallign to
     * @return Command that aligns to target pose and climbs
     */
    public Command APL1Climb(
            Pose2d targetpose) {
        return Commands.parallel(
                m_intakePivot.setAngle(() -> IntakePivotConstants.kUpperLimit),
                Commands.sequence(

                        new AutoAlign(targetpose, m_drivebase, AutoAlign.kClimbProfile)
                // ADD CLIMB COMMAND
                ));

    }

    // public Command APToAverageFuelPose() {
    // return Commands.parallel(
    // new AutoAlign(new Pose2d(m_objectVision.getAverageObjectLocation().get(), new
    // Rotation2d()), m_drivebase),
    // fuelIntake()
    // );
    // }

    public Command APToBestCluster() {
        return Commands.deferredProxy(
                () -> {
                    Optional<Cluster> clusterOpt = m_objectVision.getBestCluster();
                    if (clusterOpt.isPresent()) {
                        var clusterLoc = clusterOpt.get().getCenter();
                        var at = new AutoAlign(
                                new Pose2d(clusterLoc,
                                        clusterLoc.minus(m_drivebase.state.Pose
                                                .getTranslation())
                                                .getAngle()),
                                m_drivebase, AutoAlign.kGPDProfile);
                        return Commands.deadline(
                                at,
                                fuelIntake());
                    }
                    return Commands.none();
                });
    }

    private static int tempNumberPickedUp = 0;
    private static boolean hasTurned = false;
    private BiFunction<Integer, Rectangle2d, Command> clusterChainFunction = new BiFunction<Integer, Rectangle2d, Command>() {
        public Command apply(Integer numBalls, Rectangle2d bounds) {
            var clusterOpt = m_objectVision.getBestCluster(bounds);
            if (clusterOpt.isPresent() && tempNumberPickedUp < numBalls) {
                tempNumberPickedUp += clusterOpt.get().getPieceCount();
                var clusterLoc = clusterOpt.get().getCenter();
                var at = new AutoAlign(
                        new Pose2d(clusterLoc,
                                DriveUtil.getAngleTowards(clusterLoc, m_drivebase.state.Pose.getTranslation())),
                        m_drivebase, AutoAlign.kGPDProfile);
                return Commands.deadline(at, fuelIntake()).andThen(APToClusterChain(numBalls, bounds));
            } else if (!hasTurned && tempNumberPickedUp < numBalls) {
                hasTurned = true;
                return new AutoAlign(
                        new Pose2d(m_drivebase.state.Pose.getTranslation(),
                                DriveUtil.getAngleTowards(bounds.getCenter(), m_drivebase.state.Pose)),
                        m_drivebase, AutoAlign.kGPDProfile).andThen(APToClusterChain(numBalls, bounds));
            } else {
                hasTurned = false;
                tempNumberPickedUp = 0;
                return Commands.none();
            }
        }
    };

        public Command APToClusterChain(int numberOfBalls, boolean isLeftSide) {
                return Commands.defer(
                                () -> clusterChainFunction.apply(numberOfBalls,
                                                isLeftSide ? POI.kLeftAutoBounds.get() : POI.kRightAutoBounds.get()),
                                Set.of(m_drivebase, m_intakePivot));
        }

        public Command APToClusterChain(int numberOfBalls, Rectangle2d bounds) {
                return Commands.defer(
                                () -> clusterChainFunction.apply(numberOfBalls, bounds),
                                Set.of(m_drivebase, m_intakePivot));
        }
        public Command prepL1Climb(
            Pose2d targetpose) {
        return Commands.race(
                m_intakePivot.setAngle(() -> IntakePivotConstants.kStowAngle),
                new AutoAlign(targetpose, m_drivebase, AutoAlign.kClimbProfile),
                m_climbExtension.setHeight(() -> Inches.of(8)));
    }

    public Command L1Climb() {
        return Commands.sequence(
                m_climbExtension.setHeight(() -> Inches.of(2)).withTimeout(1));
    }

    public Command finishL1Climb() {
        return Commands.sequence(
                m_climbExtension.setHeight(() -> Inches.of(8)).withTimeout(0.5));
    }

        public Command setClimber(Supplier<Angle> pivotAngle, Supplier<Distance> extensionDistance) {
                        return m_climbExtension.setHeight(extensionDistance);
        }
        public Command fuelIntake() {
                return m_intakePivot.setAngle(() -> IntakePivotConstants.kFuelIntakeAngle);

    }

    // auto hood angle command
    public Command Score() {
        return Commands.parallel(
                m_hood.autoHoodAngle(),
                Commands.parallel(
                        m_indexer.setVoltage(() -> m_robotStates.isShootReady() ? IndexerConstants.kFastVoltage : Volts.zero()),
                        m_Spindexer.setVoltage(() -> m_robotStates.isShootReady() ? runSpindexerWithReverse.get() : Volts.zero()))
                );
    }

    private static final Current kUnjamThreshold = Amps.of(40);
    public Command autoScore() {
        

        return Commands.parallel(
                m_hood.autoHoodAngle_OVERRIDE_SAFETY(),
                Commands.parallel(
                        m_indexer.setVoltage(() ->  IndexerConstants.kFastVoltage),
                        m_Spindexer.setVoltage(runSpindexerWithReverse),
                        intakeWiggle(Degrees.of(40), Degrees.of(0), 0.75)
                        ));
    }

    public Command intakeWiggle(Angle upperLimit, Angle lowerLimit, double seconds) {
        return Commands.repeatingSequence(
                m_intakePivot.setAngle(upperLimit).withTimeout(seconds),
                m_intakePivot.setAngle(lowerLimit).withTimeout(seconds));
    }
}
