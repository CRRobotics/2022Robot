// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team639.auto;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Factory for generating trajectories */
public class TrajectoryFactory {
    Map<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    public TrajectoryFactory(String pathSubDir){
        double loadStart = System.currentTimeMillis();
        File deployDirectory = Filesystem.getDeployDirectory();
        File pathDirectory = new File(deployDirectory, pathSubDir);
        File[] pathNames = pathDirectory.listFiles();

        for (File pathname : pathNames)
        {
            if (!pathname.getName().contains(".json"))
            {
                continue;
            }
            Trajectory trajectory = loadConfig(pathname);
            if (trajectory != null)
            {
                trajectories.put(pathname.getName().replace(".wpilib.json", ""), trajectory);
            }
        }
        System.out.println("Wow, trajectories loaded in " + ((System.currentTimeMillis()) - loadStart) + " milliseconds");
    }

    /**
     * Loads trajectory config from path file
     * @param trajectoryJSON Path file to load
     * @return Trajectory to be generated
     */
    private Trajectory loadConfig(File trajectoryJSON) {
        Trajectory trajectory = new Trajectory();
        try
        {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryJSON.toPath());
        }
        catch (IOException e)
        {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON.getName(),
                e.getStackTrace());
            return null;
        }
        return trajectory;
    }


    public Trajectory getTrajectory(String name)
    {
        return trajectories.get(name);
    }
}
    

