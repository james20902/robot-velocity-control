package frc.robot.Software.Drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Hardware.Drivetrain;

import java.util.Set;

public class DriveTrajectory implements Command {

    private final double TIMEOUT_SECONDS = 2;
    private final double EUCLIDEAN_TOLERANCE_METERS = .2;
    private final double ANGLE_TOLERANCE_RAD = .1;

    private Drivetrain drive;
    private RamseteController follower;

    private Trajectory traj;
    private double startTime;

    public DriveTrajectory(Drivetrain subsystem, Trajectory traj) {
        this.drive = subsystem;
        this.traj = traj;

        follower = new RamseteController();

        //crude inverse pythagorean, assume a and b are equal
        double tolerance = Math.sqrt((EUCLIDEAN_TOLERANCE_METERS * EUCLIDEAN_TOLERANCE_METERS) / 2);

        //the feedback controller will stop if the robot thinks its x meters away from the true target,
        //and y rad away from the true target.
        follower.setTolerance(new Pose2d(new Translation2d(tolerance, tolerance), new Rotation2d(ANGLE_TOLERANCE_RAD)));

        //set false to exclusively test feedforward (no pose feedback)
        follower.setEnabled(true);
    }

    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    public void execute() {
        drive.setChassisSpeed(
                follower.calculate(drive.getPose(),
                        traj.sample(startTime + Timer.getFPGATimestamp())));
    }

    @Override
    public boolean isFinished() {
        return follower.atReference() ||
                (Timer.getFPGATimestamp() - startTime >= traj.getTotalTimeSeconds() + TIMEOUT_SECONDS);
        //if feedback controller says we're done OR the expected time has passed, we're finished
    }

    @Override
    public void end(boolean interrupt) {
        drive.setChassisSpeed(new ChassisSpeeds());
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(drive);
    }
}
