package org.team639.lib.math;

import java.util.HashMap;

public class ValueFromDistance
{
    static HashMap<Integer, AngleSpeed> shootMap = new HashMap<>();
    static
    {

    }
    public static AngleSpeed getAngleSpeed(double distance)
    {
        double linearInterpolation = distance % 1.0;
        return new AngleSpeed(
                shootMap.get((int)distance).getAngle() + linearInterpolation * ((shootMap.get((int)distance + 1).getAngle() - shootMap.get((int)distance).getAngle()) / 2),
                shootMap.get((int)distance).getSpeed() + linearInterpolation * ((shootMap.get((int)distance + 1).getSpeed() - shootMap.get((int)distance).getSpeed()) / 2)
        );



    }
}
