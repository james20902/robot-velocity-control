package frc.robot.Software.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Hardware.Drivetrain;

import java.util.Set;

public class DriveTeleop implements Command {

    private Drivetrain drive;
    private Joystick input;


    public DriveTeleop(Drivetrain subsystem, Joystick input) {
        this.drive = subsystem;
        this.input = input;
    }

    @Override
    public void execute() {
        drive.setWheelSpeedsPercent(DifferentialDrive.curvatureDriveIK(input.getX(), input.getZ(), true));
        //drive.setWheelSpeedsPercent(DifferentialDrive.arcadeDriveIK(input.getX(), input.getZ(), true));
        //drive.setWheelSpeedsPercent(DifferentialDrive.tankDriveIK(input.getX(), input.getZ(), true));
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
