package frc.robot.subsystems.agitator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;

public class RealAgitatorS extends AgitatorS {

    public static class AgitatorConstants {
        // CAN IDs
        public static final int kCAN_ID = 61;
        // Motor Properties
        public static final boolean kInverted = true;
        public static final int kStatorCurrentLimit = 80;
        public static final int kSupplyCurrentLimit = 40;
        public static final int kGearRatio = 4 / 1;
        public static final NeutralModeValue kneutralMode = NeutralModeValue.Coast;
        // Setpoints
        public static final Voltage kSlowVoltage = Volts.of(6.0);
        public static final Voltage kFastVoltage = Volts.of(10.0);

        public static TalonFXConfiguration configureMotor1(TalonFXConfiguration config) {
            config.CurrentLimits.withStatorCurrentLimit(kStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true);
            config.CurrentLimits.withSupplyCurrentLimit(kSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true);
            config.Feedback.SensorToMechanismRatio = kGearRatio;

            return config;
        }
    }

    private TalonFX m_agitatorMotor = new TalonFX(AgitatorConstants.kCAN_ID, TunerConstants.kHigherBus);

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public RealAgitatorS() {
        m_agitatorMotor.getConfigurator().apply(AgitatorConstants.configureMotor1(new TalonFXConfiguration()));
        m_agitatorMotor.setNeutralMode(AgitatorConstants.kneutralMode);
    }

    @Override
    public Command setVoltage(Supplier<Voltage> volts) {
        return Commands.run(() -> m_agitatorMotor.setControl(voltageRequest.withOutput(volts.get())))
                .finallyDo(() -> m_agitatorMotor.setControl(voltageRequest.withOutput(Volts.zero())));
    }

    @Override
    public Command resetEncoder() {
        return runOnce(() -> m_agitatorMotor.setPosition(Degrees.zero())).ignoringDisable(true);
    }

    @Override
    public AngularVelocity getVelocity() {
        return m_agitatorMotor.getVelocity().getValue();
    }

    @Override
    public Current getCurrent() {
        Optional<Current> currentOptional = Optional.of(m_agitatorMotor.getSupplyCurrent().getValue());

        return currentOptional.isPresent() ? currentOptional.get() : Amps.of(-1);
    }

}
