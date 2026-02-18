package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class RobotVisualizer {

        final static double INTAKE_X = Units.inchesToMeters(11.5); // 11.5 with AP's config file
    final static double INTAKE_Z = Units.inchesToMeters(7.85); // 7.85 with AP's config file
    final static Pose3d INTAKE_PIVOT_LOCATION = new Pose3d(INTAKE_X, 0, INTAKE_Z, new Rotation3d(Degrees.of(180),Degrees.zero(),Degrees.zero()));
    final static double HOOD_X = Units.inchesToMeters(3.5);
    final static double HOOD_Z = Units.inchesToMeters(20.5);
    final static Pose3d HOOD_LOCATION = new Pose3d(HOOD_X, 0, HOOD_Z, new Rotation3d(Degrees.of(180),Degrees.zero(),Degrees.zero()));
    final static double TURRET_X = Units.inchesToMeters(0);
    final static double TURRET_Z = Units.inchesToMeters(16);
    final static Pose3d TURRET_LOCATION = new Pose3d(TURRET_X, 0, TURRET_Z, new Rotation3d(Degrees.zero(),Degrees.zero(),Degrees.of(-90)));

    private static double hoodAngleRadians = 0;

    private static Pose3d[] components = new Pose3d[] {Pose3d.kZero, Pose3d.kZero, Pose3d.kZero, Pose3d.kZero};
    private static final StructArrayPublisher<Pose3d> layoutPub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Visualizer / Components", Pose3d.struct)
            .publish();
    public static Pose3d[] getComponents() {return components;}
    public static void updateIntake(double intakeRadians){
        components[0] = INTAKE_PIVOT_LOCATION.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, intakeRadians, 0)));
        layoutPub.set(components);
    }

    public static void updateTurret(double turretRadians){
        components[2] = HOOD_LOCATION.rotateAround(TURRET_LOCATION.getTranslation(), new Rotation3d(0, 0, turretRadians)).transformBy(
            new Transform3d(0,0,0, new Rotation3d(0, hoodAngleRadians, 0))
        );
        components[1] = TURRET_LOCATION.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, turretRadians)));
        layoutPub.set(components);
    }
    public static void updateHood(double hoodRadians){
        hoodAngleRadians = -hoodRadians;
    }
}
