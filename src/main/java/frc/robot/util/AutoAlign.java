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
    public static Command defaultToAlliance(Pose2d bluePose, Rotation2d blueEntryAngle,
            CommandSwerveDrivetrain drivetrain) {
        return AllianceFlipUtil.flippedCommand(
                (pose, rotation) -> new AutoAlign(pose, rotation, drivetrain),
                bluePose, blueEntryAngle);
    }

    public static Command climbDefaultToAlliance(Pose2d bluePose, CommandSwerveDrivetrain drivetrain) {
        return AllianceFlipUtil.flippedCommand((pose) -> new AutoAlign(pose, drivetrain), bluePose);
    }

    public static Command defaultToAlliance(Pose2d bluePose, CommandSwerveDrivetrain drivetrain) {
        return AllianceFlipUtil.flippedCommand((pose) -> new AutoAlign(pose, drivetrain), bluePose);
    }

    public static class AutoAlignConstants {
        private static final double kDefaultAcceleration = 3;
        private static final double kDefaultJerk = 6;

        private static final double kClimbAcceleration = 20;
        private static final double kClimbJerk = 3;

        public static APConstraints DEFAULT_CONSTRAINTS = new APConstraints(kDefaultAcceleration, kDefaultJerk);
        public static APConstraints CLIMB_CONSTRAINTS = new APConstraints(kClimbAcceleration, kClimbJerk);

    }

    static APProfile kDefaultProfile = new APProfile(AutoAlignConstants.DEFAULT_CONSTRAINTS)
            .withErrorXY(Centimeters.of(6))
            .withErrorTheta(Degrees.of(1.5))
            .withBeelineRadius(Centimeters.of(8));

    static APProfile kClimbProfile = new APProfile(AutoAlignConstants.CLIMB_CONSTRAINTS)
            .withErrorXY(Centimeters.of(6))
            .withErrorTheta(Degrees.of(1.5))
            .withBeelineRadius(Centimeters.of(8));

    protected final Autopilot kAutopilot;

    protected final APTarget m_target;
    protected final CommandSwerveDrivetrain m_drivetrain;
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
        this(new APTarget(targetPose), drivetrain, kDefaultProfile);
    }

    /**
     * Uses default constraints, path respects entry angle
     * 
     * @param targetPose Pose2d to align to
     * @param entryAngle Entry angle to modify approach
     * @param drivetrain Drivetrain subsystem
     */
    public AutoAlign(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain) {
        this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain,
                kDefaultProfile);
    }

    /**
     * Auto allign constructor needing all parameters
     * 
     * @param targetPose  Pose2d to align to
     * @param constraints Entry angle to modify approach
     * @param drivetrain  Drivetrain subsystem
     */
    public AutoAlign(APTarget target, CommandSwerveDrivetrain drivetrain, APProfile profile) {
        this.m_target = target;
        this.m_drivetrain = drivetrain;

        kAutopilot = new Autopilot(profile);

        addRequirements(drivetrain);
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
