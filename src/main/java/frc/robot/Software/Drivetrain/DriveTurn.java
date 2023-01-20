package frc.robot.Software.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Hardware.Drivetrain;

import java.util.Set;

public class DriveTurn implements Command {

    private Drivetrain drive;

    private final PIDController controller;
    private final Rotation2d target;
    private final double TOLERANCE_RAD = .1;
    public DriveTurn(double unit, boolean is_rad) {
        target = new Rotation2d(is_rad ? unit : Units.degreesToRadians(unit));
        controller = new PIDController(1, 0, 0);
        controller.setTolerance(1, .01);
    }

    public DriveTurn(double deg) {
        this(deg, false);
    }

    @Override
    public void execute() {
        drive.setChassisSpeed(
                new ChassisSpeeds(0,
                        0,
                        controller.calculate(drive.getChassisSpeeds().omegaRadiansPerSecond)));
    }

    @Override
    public boolean isFinished(){
        return controller.atSetpoint();
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
