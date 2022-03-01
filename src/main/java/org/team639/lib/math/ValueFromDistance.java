package org.team639.lib.math;

import org.team639.RobotContainer;

public class ValueFromDistance {
    public static AngleSpeed getAngleSpeed(double distance)
    {
        if(distance > RobotContainer.shootMap.firstKey() && distance < RobotContainer.shootMap.lastKey()) {
            double floorDistance = RobotContainer.shootMap.floorKey(distance);
            double ceilingDistance = RobotContainer.shootMap.ceilingKey(distance);
            double remainder = distance % floorDistance;

            double speed = RobotContainer.shootMap.get(floorDistance).getSpeed() + remainder * (Math.abs(RobotContainer.shootMap.get(ceilingDistance).getSpeed() - RobotContainer.shootMap.get(floorDistance).getSpeed()));
            double angle = RobotContainer.shootMap.get(floorDistance).getAngle() + remainder * (Math.abs(RobotContainer.shootMap.get(ceilingDistance).getAngle() - RobotContainer.shootMap.get(floorDistance).getAngle()));
            System.out.println("WORKING!");
            return new AngleSpeed(angle, speed);
        }
        else return (distance < RobotContainer.shootMap.firstKey())?
                //THIS WORKS DONT THINK TOO HARD ABOUT IT
                //gets the angle speed of the floor or ceiling key
                new AngleSpeed(RobotContainer.shootMap.get(RobotContainer.shootMap.firstKey()).getAngle(),RobotContainer.shootMap.get(RobotContainer.shootMap.firstKey()).getSpeed()):
                new AngleSpeed(RobotContainer.shootMap.get(RobotContainer.shootMap.lastKey()).getAngle(),RobotContainer.shootMap.get(RobotContainer.shootMap.lastKey()).getSpeed());
    }

    public static AngleSpeed getAngleSpeedLinearized(double distance) {
        if(distance > RobotContainer.shootMap.firstKey() && distance < RobotContainer.shootMap.lastKey()) 
        {
                double floorDistance = RobotContainer.shootMap.floorKey(distance); 
                double ceilingDistance = RobotContainer.shootMap.ceilingKey(distance);

                double run = ceilingDistance - floorDistance;
                
                double speed = RobotContainer.shootMap.get(floorDistance).getSpeed() + (distance - floorDistance) * ((RobotContainer.shootMap.get(ceilingDistance).getSpeed() - RobotContainer.shootMap.get(floorDistance).getSpeed())/run);
                double angle = RobotContainer.shootMap.get(floorDistance).getAngle() + (distance - floorDistance) * ((RobotContainer.shootMap.get(ceilingDistance).getAngle() - RobotContainer.shootMap.get(floorDistance).getAngle())/run);

                return new AngleSpeed(angle, speed);
        }
        return distance <= RobotContainer.shootMap.firstKey() ? RobotContainer.shootMap.get(RobotContainer.shootMap.firstKey()) : RobotContainer.shootMap.get(RobotContainer.shootMap.lastKey());
    }

    public static AngleSpeed getAngleSpeedFloored(double distance) {
        double flooredVal = RobotContainer.shootMap.floorKey(distance);
        return new AngleSpeed(RobotContainer.shootMap.get(flooredVal).getAngle(),
                RobotContainer.shootMap.get(flooredVal).getSpeed());
    }

}
