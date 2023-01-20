package frc.robot.Software.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Hardware.Drivetrain;

import java.util.List;

public class DriveStraight extends DriveTrajectory {
    public DriveStraight(Drivetrain drive, double dist_meters) {
        //we can reuse the trajectory driver and make use of some vector math.
        //we have a current position and direction, so lets assume thats our current
        //position vector from 0,0. If we just add to the current magnitude, we can
        //solve for the new x,y coordinate.
        super(drive, TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d[]{
                            drive.getPose(),
                            new Pose2d(
                                    drive.getPose().getTranslation().times(dist_meters),
                                    drive.getPose().getRotation()
                            )
                        }),
                new TrajectoryConfig(3, 6)));
    }
}
