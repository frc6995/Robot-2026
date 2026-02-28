package frc.robot.util;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignFixedHeading extends AutoAlign {
    private APTarget m_realTarget;

    private Rotation2d m_heading;

    private boolean m_cardinalize = false;

    public AutoAlignFixedHeading(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain,
            Rotation2d heading) {
        super(targetPose, entryAngle, drivetrain);
        this.m_heading = heading;
        if (m_cardinalize) {
            m_realTarget = new APTarget(new Pose2d(
                    m_target.getReference().getTranslation(),
                    cardinalizeHeading(m_heading)));
        } else {
            m_realTarget = m_target;
        }
    }

    public AutoAlignFixedHeading(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, Rotation2d heading) {
        this(new APTarget(targetPose), drivetrain, heading, AutoAlign.kDefaultProfile);
    }

    public AutoAlignFixedHeading(Pose2d targetPose, Rotation2d entryAngle, CommandSwerveDrivetrain drivetrain,
            boolean cardinalize) {
        this(new APTarget(targetPose).withEntryAngle(entryAngle), drivetrain, AutoAlign.kDefaultProfile, cardinalize);
    }

    public AutoAlignFixedHeading(Pose2d targetPose, CommandSwerveDrivetrain drivetrain, boolean cardinalize) {
        this(new APTarget(targetPose), drivetrain, AutoAlign.kDefaultProfile, cardinalize);

    }

    public AutoAlignFixedHeading(APTarget target, CommandSwerveDrivetrain drivetrain, Rotation2d fixedHeading,
            APProfile profile) {
        super(target, drivetrain, profile);
        this.m_heading = fixedHeading;
    }

    public AutoAlignFixedHeading(APTarget target, CommandSwerveDrivetrain drivetrain, APProfile profile,
            boolean cardinalize) {
        super(target, drivetrain, profile);
        this.m_cardinalize = cardinalize;
        this.m_heading = drivetrain.state.Pose.getRotation();
    }

    public static Rotation2d cardinalizeHeading(Rotation2d heading) {
        double hdegrees = MathUtil.inputModulus(heading.getDegrees(), -180, 180);
        if (hdegrees >= -135 && hdegrees < -45) {
            return Rotation2d.fromDegrees(-90);
        } else if (hdegrees >= -45 && hdegrees < 45) {
            return Rotation2d.kZero;
        } else if (hdegrees >= 45 && hdegrees < 135) {
            return Rotation2d.fromDegrees(90);
        } else if (hdegrees >= 135 && hdegrees < 180) {
            return Rotation2d.k180deg;
        } else if (hdegrees >= -180 && hdegrees < -135) {
            return Rotation2d.fromDegrees(-180);
        } else {
            return Rotation2d.kZero;
        }
    }

    public static Rotation2d cardinalizeHeadingNS(Rotation2d heading) {
        double hdegrees = MathUtil.inputModulus(heading.getDegrees(), -180, 180);
        if (hdegrees >= -90 && hdegrees <= 90) {
            return Rotation2d.kZero;
        } else if (hdegrees > 90 && hdegrees <= 180) {
            return Rotation2d.k180deg;
        } else if (hdegrees < -90 && hdegrees >= -180) {
            return Rotation2d.fromDegrees(-180);
        } else {
            return Rotation2d.kZero;
        }
    }

    @Override
    public void execute() {
        swerveState = m_drivetrain.getState();
        APResult out;
        if (m_cardinalize) {
            m_realTarget = new APTarget(new Pose2d(
                    m_target.getReference().getTranslation().getX(), m_target.getReference().getY(),
                    cardinalizeHeading(m_heading)));
            out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_realTarget);
        } else {
            m_realTarget = m_target;
            out = kAutopilot.calculate(swerveState.Pose, swerveState.Speeds, m_target);
        }

        m_drivetrain.setControl(m_request
                .withVelocityX(out.vx())
                .withVelocityY(out.vy())
                .withTargetDirection(out.targetAngle()));
    }

    @Override
    public boolean isFinished() {
        return kAutopilot.atTarget(m_drivetrain.getState().Pose, m_realTarget);
    }

}
