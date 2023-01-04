// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Hardware.Ahrs;
import frc.robot.Hardware.Drivetrain;
import frc.robot.Software.Drivetrain.DriveTeleop;
import frc.robot.Software.Drivetrain.DriveTrajectory;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    Ahrs NavX;
    Drivetrain Drive;
    Joystick OperatorInterface;

    @Override
    public void robotInit() {
        OperatorInterface = new Joystick(0);

        NavX = new Ahrs();
        Drive = new Drivetrain(NavX);
        Drive.setDefaultCommand(new DriveTeleop(Drive, OperatorInterface));

        CommandScheduler.getInstance().registerSubsystem(NavX);
        CommandScheduler.getInstance().registerSubsystem(Drive);
        CommandScheduler.getInstance().enable();
    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    @Override
    public void autonomousInit() {
        Trajectory trajectory = new Trajectory();

        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().schedule(new DriveTrajectory(Drive, trajectory));
    }
    @Override
    public void autonomousPeriodic() {}
    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().enable();
    }
    @Override
    public void teleopPeriodic() {}
    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
    }
    @Override
    public void disabledPeriodic() {}
    @Override
    public void testInit() {}
    @Override
    public void testPeriodic() {}
}
