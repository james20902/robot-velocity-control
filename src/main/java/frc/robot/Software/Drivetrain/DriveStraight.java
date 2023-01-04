package frc.robot.Software.Drivetrain;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Hardware.Drivetrain;

public class DriveStraight extends DriveTrajectory {
    public DriveStraight(Drivetrain drive) {
        //we can reuse the trajectory driver and make use of some vector math.
        //we have a current position and direction, so lets assume thats our current
        //position vector from 0,0. If we just add to the current magnitude, we can
        //solve for the new x,y coordinate.
        super(drive, TrajectoryGenerator.generateTrajectory(drive.getPose(),
                null,
                drive.getPose(),
                new TrajectoryConfig(3, 6)));

    }
}
