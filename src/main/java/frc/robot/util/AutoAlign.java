package frc.robot.util;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;
import java.util.Optional;

import com.therekrab.autopilot.Autopilot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.Autos.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {
    // Static factory methods with profile parameters
    public static Command defaultToAlliance(Pose2d bluePose, Rotation2d blueEntryAngle,
            CommandSwerveDrivetrain drivetrain) {
        return defaultToAlliance(bluePose, blueEntryAngle, drivetrain, kDefaultProfile);
    }

    public static Command defaultToAlliance(Pose2d bluePose, Rotation2d blueEntryAngle,
            CommandSwerveDrivetrain drivetrain, APProfile profile) {
        return AllianceFlipUtil.flippedCommand(
                (pose, rotation) -> new AutoAlign(pose, rotation, drivetrain, profile),
                bluePose, blueEntryAngle);
    }

    public static Command climbDefaultToAlliance(Pose2d bluePose, CommandSwerveDrivetrain drivetrain) {
        return climbDefaultToAlliance(bluePose, drivetrain, kClimbProfile);
    }

    public static Command climbDefaultToAlliance(Pose2d bluePose, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        return AllianceFlipUtil.flippedCommand((pose) -> new AutoAlign(pose, drivetrain, profile), bluePose);
    }

    public static Command defaultToAlliance(Pose2d bluePose, CommandSwerveDrivetrain drivetrain) {
        return defaultToAlliance(bluePose, drivetrain, kDefaultProfile);
    }

    public static Command defaultToAlliance(Pose2d bluePose, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        return AllianceFlipUtil.flippedCommand((pose) -> new AutoAlign(pose, drivetrain, profile), bluePose);
    }

    public static class AutoAlignConstants {
        private static final double kDefaultAcceleration = 3;
        private static final double kDefaultJerk = 6;

        private static final double kClimbAcceleration = 20;
        private static final double kClimbJerk = 3;

        public static APConstraints DEFAULT_CONSTRAINTS = new APConstraints(kDefaultAcceleration, kDefaultJerk);
        public static APConstraints CLIMB_CONSTRAINTS = new APConstraints(kClimbAcceleration, kClimbJerk);

    }

    // Make profiles public so they can be accessed and modified
    public static APProfile kDefaultProfile = new APProfile(AutoAlignConstants.DEFAULT_CONSTRAINTS)
            .withErrorXY(Centimeters.of(6))
            .withErrorTheta(Degrees.of(1.5))
            .withBeelineRadius(Centimeters.of(8));

    public static APProfile kClimbProfile = new APProfile(AutoAlignConstants.CLIMB_CONSTRAINTS)
            .withErrorXY(Centimeters.of(6))
            .withErrorTheta(Degrees.of(1.5))
            .withBeelineRadius(Centimeters.of(8));

    protected final Autopilot kAutopilot;

    protected final APTarget m_target;
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final APProfile m_profile; // Store the profile being used
    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();
    protected final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(6, 0, 0); // Replace with constants later

    protected SwerveDriveState swerveState = new SwerveDriveState();

    /**
     * Uses default constraints, beeline path
     * 
     * @param targetPose Pose2d to align to
     * @param drivetrain Drivetrain subsystem
     */
    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain) {
        this(targetPose, drivetrain, kDefaultProfile);
    }

    /**
     * Uses default constraints, beeline path with custom profile
     * 
     * @param targetPose Pose2d to align to
     * @param drivetrain Drivetrain subsystem
     * @param profile    APProfile to use for this alignment
     */
    public AutoAlign(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(new APTarget(targetPose), drivetrain, profile);
    }

    /**
     * Uses default constraints, path respects entry angle
     * 
     * @param targetPose Pose2d to align to
     * @param entryAngle Entry angle to modify approach
     * @param drivetrain Drivetrain subsystem
     */
    public AutoAlign(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain) {
        this(targetPose, entryAngle, drivetrain, kDefaultProfile);
    }

    /**
     * Uses custom profile, path respects entry angle
     * 
     * @param targetPose Pose2d to align to
     * @param entryAngle Entry angle to modify approach
     * @param drivetrain Drivetrain subsystem
     * @param profile    APProfile to use for this alignment
     */
    public AutoAlign(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain, profile);
    }

    /**
     * Auto align constructor with full parameters
     * 
     * @param target     APTarget to align to
     * @param drivetrain Drivetrain subsystem
     * @param profile    APProfile to use for this alignment
     */
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this.m_target = target;
        this.m_drivetrain = drivetrain;
        this.m_profile = profile;

        kAutopilot = new Autopilot(profile);

        addRequirements(drivetrain);
    }

    /**
     * Creates a new AutoAlign with a modified version of the current profile
     * Useful for runtime adjustments
     * 
     * @param profileModifier Function to modify the profile
     */
    public AutoAlign withModifiedProfile(java.util.function.Function<APProfile, APProfile> profileModifier) {
        APProfile modifiedProfile = profileModifier.apply(m_profile);
        return new AutoAlign(m_target, m_drivetrain, modifiedProfile);
    }

    /**
     * Gets the current profile being used
     */
    public APProfile getProfile() {
        return m_profile;
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);

        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetDirection(out.targetAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_target);
    }

}