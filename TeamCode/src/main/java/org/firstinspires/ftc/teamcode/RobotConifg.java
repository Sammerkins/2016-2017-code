package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class RobotConifg
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  harvester   = null;
    public DcMotor  accelerator = null;
    public DcMotor  cap1 = null;
    public DcMotor  cap2 = null;
    public Servo    leftslapper     = null;
    public Servo    rightslapper    = null;
    public Servo    gate            = null;
    public Servo    capgate           = null;

    public GyroSensor gyro      = null;
    //public GyroSensor compass = null;
    public ColorSensor leftcolor    = null;
    public ColorSensor rightcolor    = null;
    public OpticalDistanceSensor nautilus    = null;

    public UltrasonicSensor us1 = null;
    public UltrasonicSensor us2 = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RobotConifg() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left");
        rightMotor  = hwMap.dcMotor.get("right");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        harvester   = hwMap.dcMotor.get("harvester");
        accelerator   = hwMap.dcMotor.get("accelerator");

        cap1   = hwMap.dcMotor.get("cap1");
        cap1.setDirection(DcMotor.Direction.REVERSE);
        cap2   = hwMap.dcMotor.get("cap2");
        cap2.setDirection(DcMotor.Direction.REVERSE);

        leftslapper     = hwMap.servo.get("leftslapper");
        rightslapper     = hwMap.servo.get("rightslapper");

        gate        = hwMap.servo.get("gate");

        capgate      = hwMap.servo.get("capgate");

        gyro        = hwMap.gyroSensor.get("gyro");
        //compass     = hwMap.gyroSensor.get("compass");

        leftcolor   = hwMap.colorSensor.get("leftcolor");
        rightcolor  = hwMap.colorSensor.get("rightcolor");
        nautilus  = hwMap.opticalDistanceSensor.get("nautilus");

        us1 = hwMap.ultrasonicSensor.get("us1");
        us2 = hwMap.ultrasonicSensor.get("us2"); 

        rightcolor.setI2cAddress(I2cAddr.create8bit(0x3c));
        leftcolor.setI2cAddress(I2cAddr.create8bit(0x3e));

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
