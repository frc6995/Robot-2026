// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.agitator;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AgitatorS extends SubsystemBase {
    public abstract Command setVoltage(Supplier<Voltage> volts);
    public abstract Command resetEncoder();
    @Logged
    public abstract AngularVelocity getVelocity();
    @Logged
    public abstract Current getCurrent();
}
