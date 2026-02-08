package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class RobotVisualizer {

    final static double INTAKE_X = Units.inchesToMeters(10.625);
    final static double INTAKE_Z = Units.inchesToMeters(8.875);
    final static Pose3d INTAKE_PIVOT_LOCATION = new Pose3d(INTAKE_X, 0, INTAKE_Z, Rotation3d.kZero);
    final static double HOOD_X = Units.inchesToMeters(10.625);
    final static double HOOD_Z = Units.inchesToMeters(8.875);
    final static Pose3d HOOD_LOCATION = new Pose3d(HOOD_X, 0, HOOD_Z, Rotation3d.kZero);
    final static double TURRET_X = Units.inchesToMeters(10.625);
    final static double TURRET_Z = Units.inchesToMeters(8.875);
    final static Pose3d TURRET_LOCATION = new Pose3d(TURRET_X, 0, TURRET_Z, Rotation3d.kZero);
    final static double SPINDEXER_X = Units.inchesToMeters(0);
    final static double SPINDEXER_Z = Units.inchesToMeters(0);
    final static Pose3d SPINDEXER_LOCATION = new Pose3d(SPINDEXER_X, 0, SPINDEXER_Z, Rotation3d.kZero);

    private static Pose3d[] components = new Pose3d[] {Pose3d.kZero, Pose3d.kZero, Pose3d.kZero, Pose3d.kZero};
    private static final StructArrayPublisher<Pose3d> layoutPub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Visualizer / Components", Pose3d.struct)
            .publish();
    public static Pose3d[] getComponents() {return components;}
    public static void updateIntake(double intakeRadians){
        components[0] = INTAKE_PIVOT_LOCATION.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, -intakeRadians, 0)));
        layoutPub.set(components);
    }
    public static void updateHood(double hoodRadians){
        components[1] = HOOD_LOCATION.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(hoodRadians, 0, 0)));
        layoutPub.set(components);
    }
    public static void updateTurret(double turretRadians){
        components[1] = HOOD_LOCATION.rotateAround(TURRET_LOCATION.getTranslation(), new Rotation3d(Rotation2d.fromRadians(turretRadians)));
        components[2] = TURRET_LOCATION.transformBy(new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, turretRadians)));
        layoutPub.set(components);
    }
}
