package frc.robot.subsystems.hood;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShooterController.ShooterCommand;

public abstract class HoodS extends SubsystemBase {
    public abstract Command setAngle(Supplier<Angle> angle);
    public abstract Command setVoltage(Supplier<Voltage> voltage);
    public abstract Command sysId();
    public abstract Command autoHoodAngle();
    public abstract Command resetEncoder();

    public abstract Angle applyDynamicLimits(Angle targetAngle, Pose2d pose);
    @Logged
    public abstract Angle getAutoHoodAngle();
    public abstract Optional<Angle> getSetpoint();
    @Logged
    public abstract boolean isHoodSafe();
    @Logged
    public abstract boolean isHoodReady();
      public Command runSOTF(
    java.util.function.Supplier<frc.robot.util.ShooterController.ShooterCommand> solution,
    java.util.function.Supplier<Pose2d> robotPose
  ) {
      return setAngle(() ->
          applyDynamicLimits(
              edu.wpi.first.units.Units.Degrees.of(solution.get().hoodAngleDeg()),
              robotPose.get()
          )
      );
  }

}
