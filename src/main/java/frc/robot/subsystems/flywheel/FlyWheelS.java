package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.RealFlyWheelS.FlywheelConstants;

public abstract class FlyWheelS extends SubsystemBase {
    public abstract Command setVelocity(Supplier<AngularVelocity> speed);
    public abstract Command setVoltage(Supplier<Voltage> volts);
    public abstract Command resetEncoder();
    @Logged
    public abstract AngularVelocity getVelocity();
    public abstract Optional<AngularVelocity> getSetpoint();
    @Logged
    public abstract boolean atSetpoint();
    public boolean isFlywheelSafe() {
        return getVelocity().isNear(RPM.zero(), FlywheelConstants.kTolerance);
    }
    @Logged
    public abstract Current getCurrent();
    
    public Command runSOTF(
    Supplier<frc.robot.util.ShooterController.ShooterTargetData> solution
  ) {
      return setVelocity(() ->
          edu.wpi.first.units.Units.RotationsPerSecond
              .of(solution.get().rpm / 60.0)
      );
  }
}
