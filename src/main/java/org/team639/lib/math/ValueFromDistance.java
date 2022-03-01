package org.team639.lib.math;

import org.team639.lib.Constants;

public class ValueFromDistance {
    public static AngleSpeed getAngleSpeed(double distance)
        {
            if(distance > Constants.ShooterConstants.shootMap.firstKey() && distance < Constants.ShooterConstants.shootMap.lastKey()) {
                double floorDistance = Constants.ShooterConstants.shootMap.floorKey(distance);
                double ceilingDistance = Constants.ShooterConstants.shootMap.ceilingKey(distance);
                double remainder = distance % floorDistance;

                double speed = Constants.ShooterConstants.shootMap.get(floorDistance).getSpeed() + remainder * Math.abs(Constants.ShooterConstants.shootMap.get(ceilingDistance).getSpeed() - Constants.ShooterConstants.shootMap.get(floorDistance).getSpeed());
                double angle = Constants.ShooterConstants.shootMap.get(floorDistance).getAngle() + remainder * Math.abs(Constants.ShooterConstants.shootMap.get(ceilingDistance).getAngle() - Constants.ShooterConstants.shootMap.get(floorDistance).getAngle());

                return new AngleSpeed(angle, speed);
            }
            else return (distance < Constants.ShooterConstants.shootMap.firstKey())?
                    //THIS WORKS DONT THINK TOO HARD ABOUT IT
                    //gets the angle speed of the floor or ceiling key
                    new AngleSpeed(Constants.ShooterConstants.shootMap.get(Constants.ShooterConstants.shootMap.firstKey()).getAngle(),Constants.ShooterConstants.shootMap.get(Constants.ShooterConstants.shootMap.firstKey()).getSpeed()):
                new AngleSpeed(Constants.ShooterConstants.shootMap.get(Constants.ShooterConstants.shootMap.lastKey()).getAngle(),Constants.ShooterConstants.shootMap.get(Constants.ShooterConstants.shootMap.lastKey()).getSpeed());
    }

    public static AngleSpeed linearize(double distance) {
        double lowerbound = Constants.ShooterConstants.shootMap.floorKey(distance);
        double upperbound = Constants.ShooterConstants.shootMap.floorKey(distance);

        double angRange = Math.abs(Constants.ShooterConstants.shootMap.get(upperbound).getAngle()
                - Constants.ShooterConstants.shootMap.get(lowerbound).getAngle());
        double linearizedAngle = angRange * (distance - lowerbound) / (upperbound - lowerbound)
                + Constants.ShooterConstants.shootMap.get(lowerbound).getAngle();

        double speedRange = Math.abs(Constants.ShooterConstants.shootMap.get(upperbound).getSpeed()
                - Constants.ShooterConstants.shootMap.get(lowerbound).getSpeed());
        double linearizedSpeed = speedRange * (distance - lowerbound) / (upperbound - lowerbound)
                + Constants.ShooterConstants.shootMap.get(lowerbound).getSpeed();

        return new AngleSpeed(linearizedAngle, linearizedSpeed);
    }

    public static AngleSpeed floorAngleSpeed(double distance) {
        double flooredVal = Constants.ShooterConstants.shootMap.floorKey(distance);
        return new AngleSpeed(Constants.ShooterConstants.shootMap.get(flooredVal).getAngle(),
                Constants.ShooterConstants.shootMap.get(flooredVal).getSpeed());
    }

}
