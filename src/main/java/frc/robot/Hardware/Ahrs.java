package frc.robot.Hardware;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;


//why is AHRS a subsystem? so that instead of subsystems polling the navx and getting slightly
//different values, we can just cache the value on every scheduler run, and all other subsystems
//that get a heading from AHRS will get the exact same one for a given tick.
public class Ahrs implements Subsystem {

    private static AHRS ahrs;

    private Rotation2d rotation;

    public Ahrs() {
        //create, by default we will use SPI through the rio's MXP port
        ahrs = new AHRS();
        if (!ahrs.isConnected()) {
            throw new RuntimeException("AHRS not connected to MXP port!");
        }

        //calibrate
        ahrs.calibrate();
        if (!ahrs.isMagnetometerCalibrated()) {
            throw new RuntimeException("AHRS not calibrated! run factory calibration: https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/");
        }

        //set current angle as zero
        ahrs.zeroYaw();
    }

    @Override
    public void periodic() {
        rotation = new Rotation2d(
                Units.degreesToRadians(ahrs.getFusedHeading())
        );
    }

    public Rotation2d getRotation() {
        return rotation;
    }

}
