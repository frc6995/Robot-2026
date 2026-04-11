// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.AutoCommands;
import frc.robot.autos.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.agitator.AgitatorS;
import frc.robot.subsystems.agitator.NoneAgitatorS;
import frc.robot.subsystems.agitator.RealAgitatorS;
import frc.robot.subsystems.agitator.RealAgitatorS.AgitatorConstants;
import frc.robot.subsystems.flywheel.FlyWheelS;
import frc.robot.subsystems.flywheel.RealFlyWheelS;
import frc.robot.subsystems.hood.HoodS;
import frc.robot.subsystems.hood.RealHoodS;
import frc.robot.subsystems.hood.RealHoodS.HoodConstants;
import frc.robot.subsystems.indexer.IndexerS;
import frc.robot.subsystems.indexer.RealIndexerS;
import frc.robot.subsystems.intakepivot.IntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS;
import frc.robot.subsystems.intakepivot.RealIntakePivotS.IntakePivotConstants;
import frc.robot.subsystems.intakeroller.IntakeRollerS;
import frc.robot.subsystems.intakeroller.RealIntakeRollerS;
import frc.robot.subsystems.intakeroller.RealIntakeRollerS.IntakeRollerConstants;
import frc.robot.subsystems.spindexer.RealSpindexerS;
import frc.robot.subsystems.spindexer.SpindexerS;
import frc.robot.subsystems.spindexer.RealSpindexerS.SpindexerConstants;
import frc.robot.subsystems.turret.RealTurretS;
import frc.robot.subsystems.turret.TurretS;
import frc.robot.subsystems.turret.RealTurretS.TurretConstants;
import frc.robot.subsystems.climb.climbextension.NoneClimbExtensionS;
import frc.robot.subsystems.climb.climbextension.ClimbExtensionS;
import frc.robot.subsystems.vision.detection.NoneODVision;
import frc.robot.subsystems.vision.detection.ObjectVision;
import frc.robot.util.AutoAlign;
import frc.robot.util.AutoAlignFixedHeading;
import frc.robot.util.POI;
import frc.robot.util.ShooterController;
import frc.robot.util.Telemetry;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class RobotContainer {
    public static final TelemetryVerbosity kTelemetryVerbosity = TelemetryVerbosity.LOW;

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

    private final Telemetry logger = new Telemetry();

    public static final CommandXboxController joystick = new CommandXboxController(0);

    RobotStates robotStates = new RobotStates();

//     @Logged
    public final CommandSwerveDrivetrain m_drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);

//     @Logged(name = "Flywheel")
    private final FlyWheelS m_flywheel = new RealFlyWheelS(robotStates::isIntakeDeployed);
//     @Logged(name = "Hood")
    private final HoodS m_hood = new RealHoodS(() -> m_drivetrain.state, () -> m_drivetrain.lastState, robotStates::isIntakeDeployed);
//     @Logged(name = "Indexer")
    private final IndexerS m_indexer = new RealIndexerS();
//     @Logged(name = "IntakePivot")
    private final IntakePivotS m_intakePivot = new RealIntakePivotS(robotStates::isTurretStowed, robotStates::isFlywheelSafe);
//     @Logged(name = "IntakeRoller")
    private final IntakeRollerS m_intakeRoller = new RealIntakeRollerS();
//     @Logged(name = "Spindexer")
    private final SpindexerS m_spindexer = new RealSpindexerS();
//     @Logged(name = "Agitator")
    private final AgitatorS m_agitator = new RealAgitatorS();
//     @Logged(name = "Turret")
    private final TurretS m_turret = new RealTurretS(() -> m_drivetrain.state.Pose, () -> m_drivetrain.state.Speeds, robotStates::isIntakeDeployed);
//     @Logged(name = "ClimbExtension")
    private final ClimbExtensionS m_climbExtension = new NoneClimbExtensionS();

    // @Logged(name = "ObjectDetection")
    private final ObjectVision m_objectVision = new NoneODVision(() -> m_drivetrain.state.Pose);

    private final AutoCommands m_autoCommands = new AutoCommands(m_drivetrain, m_hood, m_intakePivot,
            m_intakeRoller, m_turret, m_indexer, m_spindexer, m_agitator, m_flywheel, m_climbExtension, m_objectVision,
            robotStates);

    private final AutoFactory autoFactory;
    private final Autos autoRoutines;
    public final AutoChooser m_chooser = new AutoChooser();

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    private Trigger shootReadyTrigger = new Trigger(robotStates::isShootReady);
    private Trigger shootNotReadyTrigger = shootReadyTrigger.negate();

    public RobotContainer() {
        ShooterController.initialize(() -> m_drivetrain.state, () -> m_drivetrain.lastState,
                (pose) -> ShooterController.getAimLocation(pose));

        autoFactory = m_drivetrain.createAutoFactory();
        autoRoutines = new Autos(m_autoCommands, m_drivetrain, autoFactory, this, m_hood, m_intakePivot,
                m_intakeRoller,
                m_turret,
                m_indexer, m_spindexer, m_agitator, m_flywheel, m_objectVision);
        SmartDashboard.putData("Auto Mode", m_chooser);

        configureDefaultCommands();
        configureBindings();
        configureTriggers();

        SignalLogger.enableAutoLogging(false);
    }

    public void periodic() {
        m_objectVision.update();
    }

    private void configureDefaultCommands() {
        m_intakePivot.setDefaultCommand(
                m_intakePivot.setAngle(() -> IntakePivotConstants.kStowAngle));

        m_intakeRoller.setDefaultCommand(
                m_intakeRoller.setVoltage(() -> {
                    return robotStates.isIntakeDeployed() ? IntakeRollerConstants.kIntakeVoltage
                            : Volts.zero();
                }));

        m_turret.setDefaultCommand(
                m_turret.runSOTF(ShooterController.getInstance()::getCachedData));

        m_hood.setDefaultCommand(m_hood.setAngle(() -> Degrees.zero()));

        m_flywheel.setDefaultCommand(
                m_flywheel.runSOTF(ShooterController.getInstance()::getCachedData));
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 5.25;
                            var ySpeed = -joystick.getLeftX() * 5.25;
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
                                            rotationSpeed); // Drive
                                                            // counterclockwise
                                                            // with negative
                                                            // X (left)
                        }));

    }

        private void configureBindings() {

        // robot relative driving with D-pad
        joystick.povCenter().whileFalse(driveIntakeRelativePOV());

        // A intake toggle
        joystick.a().toggleOnTrue(m_autoCommands.fuelIntake());

        // B button align to cardinal direction
        joystick.b().whileTrue(
                m_drivetrain.applyRequest(
                        () -> {
                            var xSpeed = -joystick.getLeftY() * 4.2;
                            var ySpeed = -joystick.getLeftX() * 4.2;
                            var rotSpeed = m_drivetrain.calculateThetaPID(
                                    AutoAlignFixedHeading.cardinalizeHeading(
                                            m_drivetrain.state.Pose
                                                    .getRotation()));

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

        // joystick.x().toggleOnTrue(
        //         Commands.parallel(
        //                 m_turret.setAngle(() -> Rotation2d.kZero),
        //                 m_hood.setAngle(() -> HoodConstants.kLowerLimit),
        //                 m_flywheel.setVelocity(() -> RPM.of(1750))
        //         )
        // );

      //  joystick.x().whileTrue(new AutoAlign(POI.TRR1.get(), m_drivetrain, AutoAlign.kDefaultVelocityLimitedProfile));

        joystick.y().onTrue(m_spindexer.runUnjam());
        joystick.y().whileTrue(m_agitator.setVoltage(() -> AgitatorConstants.kFastVoltage));

        joystick.rightTrigger().whileTrue(m_autoCommands.Score());

        joystick.leftBumper().whileTrue(m_intakePivot.setAngleSlowMove(IntakePivotConstants.kWiggleUpperAngle));

        joystick.start().debounce(0.5).onTrue(
                Commands.either(
                        Commands.parallel(
                                m_turret.driveToHome()
                            ),
                        Commands.parallel(
                                m_turret.resetEncoder(),
                                m_hood.resetEncoder()
                            ),
                        DriverStation::isEnabled
                )
            );

        // Back: Set intake to Home (disabled)
        joystick.back()
                .onTrue(m_intakePivot.resetEncoder().onlyIf(DriverStation::isDisabled));

        joystick.rightBumper()
                .whileTrue(m_autoCommands.intakeWiggle(IntakePivotConstants.kWiggleUpperAngle, IntakePivotConstants.kWiggleLowerAngle));
        joystick.rightBumper().onFalse(m_intakePivot.setAngle(() -> IntakePivotConstants.kLowerLimit));

        joystick.leftTrigger()
                .whileTrue(m_intakePivot.setAngle(() -> IntakePivotConstants.kSafeAngle));
        joystick.leftTrigger().onFalse(m_intakePivot.setAngle(() -> IntakePivotConstants.kLowerLimit));

        m_drivetrain.registerTelemetry(logger::telemeterize);

    }

    private void configureTriggers() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        shootNotReadyTrigger.debounce(0.5).and(DriverStation::isTeleopEnabled).and(joystick.rightTrigger())
                .whileTrue(
                        Commands.repeatingSequence(
                                Commands.runOnce(() -> joystick.setRumble(RumbleType.kBothRumble, 0.5)),
                                Commands.waitSeconds(0.25),
                                Commands.runOnce(() -> joystick.setRumble(RumbleType.kBothRumble, 0)),
                                Commands.waitSeconds(0.25)))
                .whileFalse(
                        Commands.runOnce(() -> joystick.setRumble(RumbleType.kBothRumble, 0)));

        RobotModeTriggers.autonomous().onTrue(
                m_turret.runSOTF(ShooterController.getInstance()::getCachedData)
                        .until(() -> !DriverStation.isAutonomous()));

        RobotModeTriggers.autonomous().onTrue(
                new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        m_flywheel.setVoltage(() -> Volts.zero()))
        );
    }

    private RobotCentric m_robotCentricRequest = new RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    public Command driveIntakeRelativePOV() {
        return m_drivetrain.applyRequest(() -> {

            double pov = Units.degreesToRadians(-joystick.getHID().getPOV());
            double adjustSpeed = Units.feetToMeters(3.5); //  converted to m/s
            return m_robotCentricRequest.withVelocityX(
                    Math.cos(pov) * adjustSpeed).withVelocityY(Math.sin(pov) * adjustSpeed)
                    .withRotationalRate(-joystick.getRightX() * 2 * Math.PI);
        });
    }

    public Command getAutonomousCommand() {
        return m_chooser.selectedCommand();

    }

    public class RobotStates {
        public boolean isShootReady() {
            return isTurretReady() && isHoodReady() && isFlywheelReady();
        }

        public boolean isTurretStowed() {
            return m_turret.getAngle().isNear(TurretConstants.kStowedAngle, TurretConstants.kTolerance);
        }

        public boolean isTurretReady() {
            return m_turret.atSetpoint();
        }

        public boolean isIntakeDeployed() {
            return m_intakePivot.isIntakeDeployed();
        }

        public boolean isHoodReady() {
            return m_hood.isHoodReady();
        }

        public boolean isHoodSafe() {
            return m_hood.isHoodSafe();
        }

        public boolean isFlywheelReady() {
            return m_flywheel.atSetpoint();
        }

        public boolean isFlywheelSafe() {
            return m_flywheel.isFlywheelSafe();
        }

        public boolean isRobotMoving() {
            return m_drivetrain.state.Speeds.vxMetersPerSecond > 0.05
                    || m_drivetrain.state.Speeds.vyMetersPerSecond > 0.05
                    || m_drivetrain.state.Speeds.omegaRadiansPerSecond > Math.PI / 36;
        }
    }
}
