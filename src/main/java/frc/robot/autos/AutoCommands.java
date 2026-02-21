package frc.robot.autos;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climb.climbpivot.ClimbPivotS;
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
import frc.robot.subsystems.spindexer.RealSpindexerS.SpindexerConstants;
import frc.robot.util.AutoAlign;
import frc.robot.util.TriggerUtil;

public class AutoCommands {
        // You need these dependencies passed in
        private final CommandSwerveDrivetrain m_drivebase;
        private final Autos autos; // Reference to your Autos class
        private final HoodS m_hood;
        private final IntakePivotS m_intakePivot;
        private final IntakeRollerS m_intakeRoller;
        private final TurretS m_turret;
        private final IndexerS m_indexer;
        private final SpindexerS m_Spindexer;
        private final FlyWheelS m_flywheel;
        private final ClimbPivotS m_climbPivot;

        SwerveRequest m_intakeDriveRequest = new SwerveRequest.ApplyRobotSpeeds()
                        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
                        .withSpeeds(new ChassisSpeeds(1.6, 0.0, 0));

        public AutoCommands(CommandSwerveDrivetrain drivebase, Autos autos, HoodS hood, IntakePivotS intakePivot,
                        IntakeRollerS intakeRoller, TurretS turret, IndexerS indexer, SpindexerS spindexer,
                        FlyWheelS flyWheel, ClimbPivotS climbPivot) {
                this.m_drivebase = drivebase;
                this.autos = autos;
                this.m_hood = hood;
                this.m_intakePivot = intakePivot;
                this.m_intakeRoller = intakeRoller;
                this.m_turret = turret;
                this.m_indexer = indexer;
                this.m_Spindexer = spindexer;
                this.m_flywheel = flyWheel;
                this.m_climbPivot = climbPivot;
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
         * @param driveTime            Time to drive forward collecting fuel
         * @return command that intakes from the center line
         */
        public Command APToIntake(
                        Pose2d helpPose,
                        Distance helpPoseTolerance,
                        Pose2d intakePose,
                        Rotation2d intakePoseEntryAngle,
                        Distance intakePoseTolerance,
                        Time driveTime) {

                return Commands.deadline(
                                Commands.sequence(

                                                new AutoAlign(helpPose, m_drivebase).until(
                                                                TriggerUtil.isWithinRadius(
                                                                                () -> helpPose.getTranslation(),
                                                                                () -> m_drivebase.state.Pose,
                                                                                () -> helpPoseTolerance)),
                                                new AutoAlign(intakePose, intakePoseEntryAngle, m_drivebase).until(
                                                                TriggerUtil.isWithinRadius(
                                                                                () -> intakePose.getTranslation(),
                                                                                () -> m_drivebase.state.Pose,
                                                                                () -> intakePoseTolerance)),

                                                (m_drivebase.applyRequest(() -> m_intakeDriveRequest)
                                                                .withTimeout(driveTime))),
                                Commands.parallel(fuelIntake(),
                                                m_hood.setAngle(() -> HoodConstants.kLowerLimit)
                                                ));
        }

        /**
         * Command that returns robot from intake that starts with Choreo path
         * 
         * @param choreoCommand        Choreo command to start with
         * @param helpPose             Pose to invoke tolerance (radius) from for
         *                             stoping the choreo path
         * @param helpPoseTolerance    Tolerance (radius) from help pose for stoping the
         *                             choreo path
         * @param intakePose           Pose to go through before slowDriveForward
         * @param intakePoseEntryAngle
         * @param intakePoseTolerance  Minimum radius for robot to be in from intakePose
         *                             that triggers the next command
         * @param driveTime            Time to drive forward collecting fuel
         * @return Command that returns robot from intake that starts with Choreo path
         */
        public Command choreoToIntake(
                        Command choreoCommand,
                        Pose2d helpPose,
                        Distance helpPoseTolerance,
                        Pose2d intakePose,
                        Rotation2d intakePoseEntryAngle,
                        Distance intakePoseTolerance,
                        Time driveTime) {
                return Commands.deadline(
                                Commands.sequence(
                                                m_hood.setAngle(() -> HoodConstants.kLowerLimit),
                                                m_turret.driveToHome(),
                                                Commands.waitUntil(() -> m_hood.isHoodSafe()),
                                                choreoCommand.until(
                                                                TriggerUtil.isWithinRadius(
                                                                                () -> helpPose.getTranslation(),
                                                                                () -> m_drivebase.state.Pose,
                                                                                () -> helpPoseTolerance)),
                                                new AutoAlign(intakePose, intakePoseEntryAngle, m_drivebase).until(
                                                                TriggerUtil.isWithinRadius(
                                                                                () -> intakePose.getTranslation(),
                                                                                () -> m_drivebase.state.Pose,
                                                                                () -> intakePoseTolerance)),

                                                (m_drivebase.applyRequest(() -> m_intakeDriveRequest)
                                                                .withTimeout(driveTime))),
                                fuelIntake());

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
                                Commands.waitUntil(() -> m_hood.isHoodSafe()),
                                new AutoAlign(helpPose, helpPoseEntryAngle, m_drivebase).until(
                                                TriggerUtil.isWithinRadius(
                                                                () -> helpPose.getTranslation(),
                                                                () -> m_drivebase.state.Pose,
                                                                () -> helpPoseTolerance)),
                                new AutoAlign(targetpose, targetPoseEntryAngle, m_drivebase));

        }

        /**
         * Command that returns robot from intake that starts with Choreo path
         * 
         * @param choreoCommand        Choreo command to start with
         * @param helpPose             Pose to invoke tolerance (radius) from for
         *                             stoping the choreo path
         * @param helpPose             Tolerance (radius) from help pose for stoping the
         *                             choreo path
         * @param targetpose           Target pose to autoallign to
         * @param targetPoseEntryAngle
         * @return Command that returns robot from intake that starts with a Choreo path
         */
        public Command choreoBackFromIntake(
                        Command choreoCommand,
                        Pose2d helpPose,
                        Distance helpPoseTolerance,
                        Pose2d targetpose,
                        Rotation2d targetPoseEntryAngle) {
                return Commands.sequence(
                                Commands.waitUntil(() -> m_hood.isHoodSafe()),
                                choreoCommand.until(
                                                TriggerUtil.isWithinRadius(
                                                                () -> helpPose.getTranslation(),
                                                                () -> m_drivebase.state.Pose,
                                                                () -> helpPoseTolerance)),
                                new AutoAlign(targetpose, targetPoseEntryAngle, m_drivebase));

        }

        /**
         * Command that drives robot to ladder starting with Choreo command and climbs
         * to L1
         * 
         * @param choreoCommand        Choreo command to start with
         * @param helpPose             Pose to invoke tolerance (radius) from for
         *                             stoping the choreo path
         * @param helpPose             Tolerance (radius) from help pose for stoping the
         *                             choreo path
         * @param targetpose           Target pose to autoallign to
         * @param targetPoseEntryAngle
         * @return Command that returns robot from intake that starts with a Choreo path
         */
        public Command choreoL1Climb(
                        Command choreoCommand,
                        Pose2d helpPose,
                        Distance helpPoseTolerance,
                        Pose2d targetpose) {
                return Commands.parallel(
                                m_intakePivot.setAngle(() -> IntakePivotConstants.kUpperLimit),
                                Commands.sequence(

                                                choreoCommand.until(
                                                                TriggerUtil.isWithinRadius(
                                                                                () -> helpPose.getTranslation(),
                                                                                () -> m_drivebase.state.Pose,
                                                                                () -> helpPoseTolerance)),
                                                new AutoAlign(targetpose, m_drivebase)
                                // ADD CLIMB COMMAND
                                ));

        }

        public Command fuelIntake() {
                return Commands.parallel(
                                m_intakePivot.setAngle(() -> IntakePivotConstants.kLowerLimit),
                                m_intakeRoller.setVoltage(() -> IntakeRollerConstants.kIntakeVoltage));
        }

        // auto hood angle command
        public Command Score() {
                return Commands.sequence(
                                m_hood.autoHoodAngle(),
                                Commands.waitUntil(() -> m_hood.isHoodReady() && m_turret.atSetpoint()
                                                && m_flywheel.atSetpoint()),
                                Commands.parallel(
                                                m_indexer.setVoltage(() -> IndexerConstants.kIntakeVoltage),
                                                m_Spindexer.setVelocity(
                                                                () -> SpindexerConstants.kVelocity)));
        }
        public Command climbL1() {
            return Commands.sequence(
                m_climbPivot.setAngle(()->Degrees.of(30))
            );
        }
}
