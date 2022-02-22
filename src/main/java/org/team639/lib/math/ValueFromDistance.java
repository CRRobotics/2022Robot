package org.team639.lib.math;

import java.util.HashMap;

public class ValueFromDistance
{
    static HashMap<Integer, AngleSpeed> shootMap = new HashMap<>();
    static
    {
        shootMap.put();
        shootMap.put();
        shootMap.put();
        shootMap.put();
        shootMap.put();
        shootMap.put();
        shootMap.put();
    }
    public static AngleSpeed getAngle(double distance)
    {
        double linearInterpolation = distance % 1;
        return new AngleSpeed(
                shootMap.get((int)distance).getAngle() + linearInterpolation * ((shootMap.get((int)distance).getAngle() + shootMap.get((int)distance + 1).getAngle()) / 2),
                shootMap.get((int)distance).getSpeed() + linearInterpolation * ((shootMap.get((int)distance).getSpeed() + shootMap.get((int)distance + 1).getSpeed()) / 2));



    }
}
