package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class RealledS extends SubsystemBase {

    private final int CANdleID = 1;

    public static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

 private final CANdle m_candle = new CANdle(CANdleID, TunerConstants.kCANBus);



     public RealledS() {
        /* Configure CANdle */
        var cfg = new CANdleConfiguration();
        /*set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
         /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(cfg);

    }  



    public Command setColorCommand() {
    return run(() -> m_candle.setControl(new SolidColor(0, 60).withColor(kGreen)));
}

    
}

