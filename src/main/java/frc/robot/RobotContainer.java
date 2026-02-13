// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.autos.AutoCommands;
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.FlyWheelS;
import frc.robot.subsystems.flywheel.NoneFlyWheelS;
import frc.robot.subsystems.flywheel.RealFlyWheelS;
import frc.robot.subsystems.flywheel.RealFlyWheelS.FlywheelConstants;
import frc.robot.subsystems.hood.HoodS;
import frc.robot.subsystems.hood.NoneHoodS;
import frc.robot.subsystems.hood.RealHoodS;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
import frc.robot.subsystems.indexer.IndexerS;
import frc.robot.subsystems.indexer.NoneIndexerS;
import frc.robot.subsystems.indexer.RealIndexerS;
import frc.robot.subsystems.intakepivot.IntakePivotS;
import frc.robot.subsystems.intakepivot.NoneIntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS.IntakePivotConstants;
import frc.robot.subsystems.intakeroller.IntakeRollerS;
import frc.robot.subsystems.intakeroller.NoneIntakeRollerS;
import frc.robot.subsystems.intakeroller.RealIntakeRollerS;
import frc.robot.subsystems.spindexer.NoneSpindexerS;
import frc.robot.subsystems.spindexer.RealSpindexerS;
import frc.robot.subsystems.spindexer.SpindexerS;
import frc.robot.subsystems.turret.NoneTurretS;
import frc.robot.subsystems.turret.RealTurretS;
import frc.robot.subsystems.turret.TurretS;
import frc.robot.subsystems.vision.RealVision;
import frc.robot.util.AutoAlignFixedHeading;
import frc.robot.util.Telemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                  // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                     // motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = new CommandSwerveDrivetrain(
        true,
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight
    );

    private final FlyWheelS m_flywheel = new NoneFlyWheelS();
//     private final HoodS m_hood = new RealHoodS(() -> m_drivetrain.state.Pose, () -> m_drivetrain.state.Speeds);
    private final HoodS m_hood = new NoneHoodS();
    private final IndexerS m_indexer = new NoneIndexerS();
    private final IntakePivotS m_intakePivot = new NoneIntakePivotS();
    private final IntakeRollerS m_intakeRoller = new NoneIntakeRollerS();
    private final SpindexerS m_spindexer = new NoneSpindexerS();
//     private final TurretS m_turret = new RealTurretS(() -> m_drivetrain.state.Pose, () -> m_drivetrain.state.Speeds, ()-> m_intakePivot.isIntakeDeployed());
    private final TurretS m_turret = new NoneTurretS();
    private final AutoCommands m_AutoCommands = new AutoCommands(m_drivetrain, null, m_hood, m_intakePivot,
            m_intakeRoller, m_turret, m_indexer, m_spindexer, m_flywheel);

    private final AutoFactory autoFactory;
    private Mechanism2d VISUALIZER;
    private final Autos autoRoutines;
    public final AutoChooser m_chooser = new AutoChooser();

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    public RobotContainer() {

        m_drivetrain.resetOdometry(new Pose2d());
        VISUALIZER = logger.MECH_VISUALIZER;

        SmartDashboard.putData("Visualzer", VISUALIZER);

        autoFactory = m_drivetrain.createAutoFactory();
        autoRoutines = new Autos(m_drivetrain, autoFactory, this, m_hood, m_intakePivot, m_intakeRoller, m_turret,
                m_indexer, m_spindexer, m_flywheel);
        SmartDashboard.putData("Auto Mode", m_chooser);
        configureBindings();

    }

     
    public double xButtonPressedTime = 0;
    public boolean intakeState = false;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 4.2;
                            var ySpeed = -joystick.getLeftX() * 4.2;
                            var rotationSpeed = -joystick.getRightX() * 2 * Math.PI;

                            if (DriverStation.isAutonomous()) {
                               
                                return m_driveRequest.withVelocityX(0).withVelocityY(0)
                                        .withRotationalRate(0);
                            }
                            return m_driveRequest
                                    .withVelocityX(
                                            xSpeed) // Drive forward with
                                                    // negative Y (forward)
                                    .withVelocityY(
                                            ySpeed) // Drive left with
                                                    // negative X (left)
                                    .withRotationalRate(
                                            rotationSpeed);
                        } // Drive counterclockwise with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // A intake toggle
        joystick.a().whileTrue(
                m_intakePivot.setAngle(() -> IntakePivotConstants.kFuelIntakeAngle));
        joystick.y().whileTrue(
                m_intakePivot.setAngle(() -> IntakePivotConstants.kStowAngle));
                // m_intakeroller.setVoltage(() -> intakeState ? IntakeRollerConstants.kIntakeVoltage : Volts.of(0))));

        // B button align to cardinal direction
        joystick.b().whileTrue(
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 4.2;
                            var ySpeed = -joystick.getLeftX() * 4.2;
                            var rotSpeed = m_drivetrain.calculateThetaPID(
                                    AutoAlignFixedHeading.cardinalizeHeading(m_drivetrain.state.Pose.getRotation()));

                            return m_driveRequest
                                    .withVelocityX(
                                            xSpeed) // Drive forward with
                                                    // negative Y (forward)
                                    .withVelocityY(
                                            ySpeed) // Drive left with
                                                    // negative X (left)
                                    .withRotationalRate(
                                            rotSpeed);
                        } // Drive counterclockwise with negative X (left)
                ));

        // right trigger hold to score
        joystick.rightTrigger().whileTrue(m_AutoCommands.Score());
        m_flywheel.setDefaultCommand(m_flywheel.setVelocity(()->FlywheelConstants.kShootSpeed));
        m_turret.setDefaultCommand(m_turret.aimAtHub());
        m_hood.setDefaultCommand(m_hood.setAngle(()->HoodConstants.kLowerLimit));

        // start button home turret
        joystick.start()
                .onTrue(m_turret.driveToHome().onlyIf(() -> DriverStation.isEnabled()));
        // select button home all, reset on disable
        joystick.back().onTrue(Commands.either(
                Commands.parallel(
                        m_turret.driveToHome(),
                        m_flywheel.resetEncoder(),
                        m_spindexer.resetEncoder(),
                        m_intakeRoller.resetEncoder(),
                        m_intakePivot.resetEncoder(),
                        m_indexer.resetEncoder(),
                        m_hood.resetEncoder()),
                Commands.parallel(
                        m_turret.resetEncoder(),
                        m_flywheel.resetEncoder(),
                        m_spindexer.resetEncoder(),
                        m_intakeRoller.resetEncoder(),
                        m_intakePivot.resetEncoder(),
                        m_indexer.resetEncoder(),
                        m_hood.resetEncoder()),
                () -> DriverStation.isEnabled()));

        joystick.a().onTrue(m_turret.driveToHome());
        joystick.x().whileTrue(m_hood.autoHoodAngle());


        joystick.povCenter().whileFalse(driveIntakeRelativePOV());

        // 😢pain
        m_drivetrain.registerTelemetry(logger::telemeterize);

        RobotModeTriggers.autonomous().onTrue(Commands.deferredProxy(() -> m_turret.driveToHome()));

    }

    private RobotCentric m_robotCentricRequest = new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    public Command driveIntakeRelativePOV() {
        return m_drivetrain.applyRequest(() -> {
            double pov = Units.degreesToRadians(-joystick.getHID().getPOV());
            double adjustSpeed = Units.feetToMeters(3); // m/s
            return m_robotCentricRequest.withVelocityX(
                    Math.cos(pov) * adjustSpeed).withVelocityY(
                            Math.sin(pov) * adjustSpeed)
                    .withRotationalRate(
                            -joystick.getRightX() * 2
                                    * Math.PI);
        });
    }

    public Command getAutonomousCommand() {
        return m_chooser.selectedCommand();

    }

    
}
