package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitUtil {
    public static <U extends Unit, M extends Measure<U>> M max(M a, M b) {
        return a.gte(b) ? a : b;
    }
    public static <U extends Unit, M extends Measure<U>> M min(M a, M b) {
        return a.lte(b) ? a : b;
    }
    public static <U extends Unit, M extends Measure<U>> M clamp(M value, M low, M high) {
        return max(low, min(value, high));
    }
    
}
