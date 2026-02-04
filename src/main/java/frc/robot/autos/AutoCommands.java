package frc.robot.autos;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodS;
import frc.robot.subsystems.HoodS.hoodConstants;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.TurretS;
import frc.robot.util.AutoAlign;
import frc.robot.util.POI;
import frc.robot.util.TriggerCommand;
import frc.robot.util.TriggerUtil;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.wpilibj2.command.Commands.*; // Static import for WPILib Commands

public class AutoCommands {
    // You need these dependencies passed in
    private final CommandSwerveDrivetrain m_drivebase;
    private final Autos autos; // Reference to your Autos class
    /*private final HoodS m_hood;*/
    private final IntakePivotS m_intakePivot;
    private final IntakeRollerS m_intakeRoller;
    private final TurretS m_turret;

    SwerveRequest m_LIntakeRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
            .withSpeeds(new ChassisSpeeds(0.5, 0.0, 0));

    SwerveRequest m_RIntakeRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.Velocity)
            .withSpeeds(new ChassisSpeeds(0.5, 0., 0));

    public AutoCommands(CommandSwerveDrivetrain drivebase, Autos autos, /*HoodS hood,*/ IntakePivotS intakePivot,
            IntakeRollerS intakeRoller, TurretS turret) {
        this.m_drivebase = drivebase;
        this.autos = autos;
        /*this.m_hood = hood;*/
        this.m_intakePivot = intakePivot;
        this.m_intakeRoller = intakeRoller;
        this.m_turret = turret;
    }

    public Command autoToIntake(BooleanSupplier isLeftSide, Pose2d intakePose, double driveTime) {
        return Commands.sequence(
                //TO DO: FIX WITH SIDE OF FIELD
                    isLeftSide.getAsBoolean() ? new AutoAlign(POI.HELPL1.get(), POI.HELPL1Entry.get(), m_drivebase).until(
                                TriggerUtil.isWithinRadius(() -> POI.HELPL1.get().getTranslation(), () -> m_drivebase.state.Pose, () -> Meters.of(1.0)))
                        : new AutoAlign(POI.HELPR1.get(), POI.HELPR1Entry.get(), m_drivebase).until(
                                TriggerUtil.isWithinRadius(() -> POI.HELPR1.get().getTranslation(), () -> m_drivebase.state.Pose, () -> Meters.of(1.0))),
                     
                    new AutoAlign(intakePose, m_drivebase).until(
                            TriggerUtil.isWithinRadius(() -> intakePose.getTranslation(), () -> m_drivebase.state.Pose, () -> Meters.of(0.3))),
                           
                    (m_drivebase.applyRequest(() -> m_RIntakeRequest).withTimeout(driveTime)
        ));

    }

    // public Command autoBackFromIntake(BooleanSupplier isLeftSide) {
    //     if (isLeftSide.getAsBoolean()) {
    //         return Commands.sequence(
    //                 m_hood.setAngle(hoodConstants.kStowAngle), // replace with threshold command
    //                 new AutoAlign(POI.TRL1.get(), POI.TRL1Entry.get(), m_drivebase));
    //     } else {
    //         return Commands.sequence(
    //                 m_hood.setAngle(hoodConstants.kStowAngle), // replace with threshold command
    //                 new AutoAlign(POI.TRR1.get(), POI.TRR1Entry.get(), m_drivebase));
    //     }

    // }

    public Command fuelIntake() {
        return Commands.parallel(
                m_intakePivot.setAngle(IntakePivotS.intakeConstants.kCW),
                m_intakeRoller.setVoltage(IntakeRollerS.rollerConstants.kIntakeVoltage));
    }
}
