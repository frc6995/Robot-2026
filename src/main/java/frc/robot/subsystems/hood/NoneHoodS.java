package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.ShooterController.ShooterTargetData;

public class NoneHoodS extends HoodS{

    @Override
    public Command setAngle(Supplier<Angle> angle) {
        return Commands.none();
    }

    @Override
    public Command setVoltage(Supplier<Voltage> voltage) {
        return Commands.none();
    }

    @Override
    public Command sysId() {
        return Commands.none();
    }

    @Override
    public Command autoHoodAngle() {
       return Commands.none();
    }

    @Override
    public Command resetEncoder() {
        return Commands.none();
    }

    @Override
    public Angle applyDynamicLimits(Angle targetAngle, Pose2d pose) {
        return Degrees.zero();
    }

    @Override
    public Optional<Angle> getSetpoint() {
        return Optional.of(Degrees.zero());
    }

    @Override
    public boolean isHoodSafe() {
        return true;
    }

    @Override
    public boolean isHoodReady() {
        return true;
    }
    
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        defaultCommand.addRequirements(this);
        super.setDefaultCommand(defaultCommand);
    }

    @Override
    public Command runSOTF(Supplier<ShooterTargetData> dataSupplier) {
        return Commands.none();
    }
}
