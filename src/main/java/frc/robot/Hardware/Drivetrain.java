package frc.robot.Hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain implements Subsystem {

    private final double NOMINAL_VOLTAGE = 12;
    private final double MAX_SPEED_METERS = 3;

    //counts per revolution for encoder
    //for a mag encoder, this is 4096. SysID gives you other CPR values for other encoders as well.
    private final short ENCODER_CPR = 4096;

    private final double WHEEL_RADIUS_METERS = .2;
    private final double WHEEL_CIRCUMFERENCE_METERS = 2*Math.PI*WHEEL_RADIUS_METERS;

    private final TalonSRX frontLeft;
    private final TalonSRX frontRight;
    private final TalonSRX backLeft;
    private final TalonSRX backRight;

    private final Ahrs NavxInstance;

    @Config.PIDController
    private final PIDController leftPID;
    @Config.PIDController
    private final PIDController rightPID;

    private final SimpleMotorFeedforward ff;

    @Log.ToString
    private final DifferentialDriveWheelSpeeds currentVelocity;
    @Log.ToString
    private DifferentialDriveWheelSpeeds velocitySetpoint;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    public Drivetrain(Ahrs navx) {
        frontLeft = new TalonSRX(1);
        frontRight = new TalonSRX(2);
        backLeft = new TalonSRX(3);
        backRight = new TalonSRX(4);

        //current limit setup, setting these correctly will prevent brownout.
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 40; // amps the motor can hit
        config.peakCurrentDuration = 1000; // how long to sustain max in ms
        config.continuousCurrentLimit = 30; // drop down if max sustained for peakDuration

        // apply current limiting
        frontLeft.configAllSettings(config);
        frontRight.configAllSettings(config);
        backLeft.configAllSettings(config);
        backRight.configAllSettings(config);

        frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        this.NavxInstance = navx;

        leftPID = new PIDController(0, 0, 0);
        rightPID = new PIDController(0, 0, 0);

        currentVelocity = new DifferentialDriveWheelSpeeds();
        velocitySetpoint = new DifferentialDriveWheelSpeeds();

        // used to track position based on gyro and encoder position
        odometry = new DifferentialDriveOdometry(navx.getRotation());

        // utility class to solve the overall robot motion vector from the left and right velocities.
        kinematics = new DifferentialDriveKinematics(.5);

        // Linear regression calculator, predicts the amount of voltage needed to be applied
        // to get a desired velocity in m/s. USE SYSID CONSTANTS.
        ff = new SimpleMotorFeedforward(0, 0, 0);
    }

    // Our periodic function will constantly control the drivetrain's two motion vectors.
    @Override
    public void periodic() {
        //getSelectedSensorVelocity returns ticks per 100ms aka ticks per .1 seconds
        //so if we just want seconds, multiply by 10.
        //10 ticks every .1 seconds is 100 ticks every whole second.
        double TalonToSecondsLeft = frontLeft.getSelectedSensorVelocity() * 10;
        double TalonToSecondsRight = frontRight.getSelectedSensorVelocity() * 10;

        currentVelocity.leftMetersPerSecond = encoderToMeters(TalonToSecondsLeft);
        currentVelocity.rightMetersPerSecond = encoderToMeters(TalonToSecondsRight);

        // ff.calculate will use the linear regression we calculated in SysID and guess how much voltage
        // is needed to achieve our desired velocity. PID is used to fill in the gaps because feedforward
        // only takes us so far.
        frontLeft.set(ControlMode.PercentOutput,
                (ff.calculate(velocitySetpoint.leftMetersPerSecond) +
                        leftPID.calculate(currentVelocity.leftMetersPerSecond)) / NOMINAL_VOLTAGE);
        frontRight.set(ControlMode.PercentOutput,
                (ff.calculate(velocitySetpoint.rightMetersPerSecond) +
                        rightPID.calculate(currentVelocity.rightMetersPerSecond)) / NOMINAL_VOLTAGE);

        // whatever the front two do, follow.
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        odometry.update(NavxInstance.getRotation(),
                encoderToMeters(frontLeft.getSelectedSensorPosition()),
                encoderToMeters(frontRight.getSelectedSensorPosition()));
    }

    private double encoderToMeters(double encoderUnits) {
        //convert ticks to revolutions.
        //if we go 4096 ticks, we're going one revolution.
        double TicksToRevolutions = encoderUnits / ENCODER_CPR;

        //convert revolutions per second to meters.
        //if we assume 0 slip and perfect energy transfer, we can say one revolution is our wheel circumference.
        return TicksToRevolutions * WHEEL_CIRCUMFERENCE_METERS;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setChassisSpeed(ChassisSpeeds chassis) {
        setWheelSpeeds(kinematics.toWheelSpeeds(chassis));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(currentVelocity);
    }

    public void setWheelSpeeds(DifferentialDriveWheelSpeeds target) {
        this.velocitySetpoint = target;
    }

    public void setWheelSpeedsPercent(DifferentialDrive.WheelSpeeds speeds) {
        // the WheelSpeeds object returns a value from -1 to 1,
        // if we want the drivetrain to be controlled exclusively through velocity,
        // we just treat it as a percentage of the theoretical max speed.
        setWheelSpeeds(new DifferentialDriveWheelSpeeds(speeds.left * MAX_SPEED_METERS,
                                                        speeds.right * MAX_SPEED_METERS));
    }

}
