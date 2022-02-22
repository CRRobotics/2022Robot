package org.team639.lib.math;

import org.team639.lib.Constants;

public class ValueFromDistance {
    public static AngleSpeed getAngleSpeed(double distance) {
        double linearInterpolation = distance % 1.0;
        return new AngleSpeed(
                Constants.ShooterConstants.shootMap.get((int) distance).getAngle()
                        + linearInterpolation * (Constants.ShooterConstants.shootMap.get((int) distance + 1).getAngle()
                                - Constants.ShooterConstants.shootMap.get((int) distance).getAngle()),
                Constants.ShooterConstants.shootMap.get((int) distance).getSpeed()
                        + linearInterpolation * (Constants.ShooterConstants.shootMap.get((int) distance + 1).getSpeed()
                                - Constants.ShooterConstants.shootMap.get((int) distance).getSpeed()));
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
