/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConifg;

/* VERSION HISTORY
    1 - 10/19/16 - Initial Version
    2 - 10/19/16 - Converted tank drive to dpad drive
                   Added speed adjustment
    3 - 10/19/16 - Moved drive control over to the left joystick
                   Added gyro
    4 - 10/31/16 - Initial conversion to map based drive
                   Added basic slapper controls
    5 - 11/09/16 - Removed map based drive until compass gyro is aquired
    6 - 11/28/16 - Added second joystick
                   Added harvester controls
                   Added harvester stutter
    7 - 12/19/16 - Added gyro correction to driving
                   Removed stutter for harvester as it was outdated
                   General cleaning and commenting
    8 - 01/04/17 - Added second beacon trigger
                   Added second color sensor
    9 - 01/25/17 - Gave controller two control over side beacon triggers
                   Adjusted accelerator controls
                   Added gate controls
                   Added accelerator lock when gate is open
    10 - 01/28/17 - Added automated accelerator controls
    11 - 02/11/17 - STATE TOURNAMENT
    12 - 03/31/17 - SUPER REGIONALS
    13 - 04/10/17 - Cleaned up codes
                    Added smooth acceleration and deceleration
 */


@TeleOp(name="Drive: Drivecode", group="Drive")
//@Disabled

public class Drivecode extends LinearOpMode {
    boolean gyroset = false;
    double gyrozero = 0;

    double gyroval = 0;
    double gyroLastReset = 0;

    double leftmotorS;
    double rightmotorS;

    //drive vars
    double[] speeds = {0.5, 0.75, 1.0};
    int speed = 2;

    boolean lastSpeed = false;
    boolean accelerateMono = true;
    double accelerateStart = 0;
    boolean decelerateInitMono = true;
    double decelerateStart = 0;
    boolean decelerate = false;
    boolean decelerateMono = true;

    boolean black = false;
    boolean fire = false;
    boolean fireCommand = false;
    boolean readyToFire = false;
    double launchTime = -1;
    double extraSpinStart = 0;
    boolean extraSpinReset = true;
    boolean cock = false;
    int shotCount = 0;

    boolean reverseMono = false;
    boolean reverse = false;

    boolean holdPosition = false;

    boolean lastGate = false;
    boolean gateOpen = true;

    // cap gate variables
    boolean buttonA_pressed = false;
    double capgateservo = 0.1;

    RobotConifg         robot     = new RobotConifg();
    private ElapsedTime runtime   = new ElapsedTime();

    double harvesterspeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.nautilus.enableLed(true);

        telemetry.addData("Jarvis", "Good luck sir!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            Accelerator();
            CapBall();
            CapGate();
            Wings();
            Harvester();
            DriveTrain();
            Telemetry();

            //CLOSING INFORMATION
            telemetry.update();
            robot.waitForTick(40);
            idle();
        }
    }


    public void Accelerator(){
        double Aspeed = 0;

        fireCommand = gamepad2.a;

        black = robot.nautilus.getLightDetected() > 0.044;

        if (fireCommand && readyToFire){
            harvesterspeed = 0;
            gateOpen = false;
            launchTime = runtime.seconds();
            readyToFire = false;
        }
        if (runtime.seconds() > launchTime + 0.4 && runtime.seconds() < launchTime + 0.5) {
            fire = true;
            cock = false;
        }


        if (black && !fire){
            if (runtime.seconds() > extraSpinStart + 0.05) {// + 0.05
                Aspeed = 0;
                cock = true;
            }
            else
                Aspeed = 1;

            if (runtime.seconds() > extraSpinStart + 0.9 && gateOpen)
                readyToFire = true;

            if (extraSpinReset) {
                extraSpinStart = runtime.seconds();
                extraSpinReset = false;
            }
        }
        else if (black && fire){
            Aspeed = 1;
            extraSpinReset = true;
        }
        else if (!black && fire){
            Aspeed = 1;
            fire = false;
            shotCount++;
        }
        else if (!black && !fire && !cock){
            Aspeed = 1;
            gateOpen = true;
        }

        robot.accelerator.setPower(Aspeed);
        if (gateOpen)
            robot.gate.setPosition(0);
        else
            robot.gate.setPosition(0.65);
    }


    public void Harvester(){
        //speed declaration
        if (gamepad2.dpad_up)
            harvesterspeed = -0.75;
        else if (gamepad2.dpad_down)
            harvesterspeed = 0.75;
        else if (gamepad2.dpad_left)
            harvesterspeed = 0;

        robot.harvester.setPower(harvesterspeed);
    }


    public void CapBall(){
        double capPower1 = 0;
        double capPower2 = 0;

        if (holdPosition){

        }
        else{
            if (gamepad1.left_bumper){
                capPower1 = -1;
                capPower2 = -1;
            }
            else if (gamepad1.left_trigger > 0.5){
                capPower1 = 0.1;
                capPower2 = 0.1;
            }
        }

        robot.cap1.setPower(capPower1);
        robot.cap2.setPower(capPower2);
    }


    public void CapGate(){

        /* check if button has just been pressed */
        if (gamepad1.a && !buttonA_pressed) {
            /* this should only happen the first time we notice the button press */
            buttonA_pressed = true;
            if (capgateservo==0.1)
                capgateservo = 0.3;
            else if (capgateservo==0.3)
                capgateservo = 0.7;
            else
                capgateservo = 0.1;
        }
        if (!gamepad1.a)
            buttonA_pressed = false;

        robot.capgate.setPosition(capgateservo);
    }


    public void Wings(){
        double servoval = 0;
        if (gamepad1.y || gamepad2.right_bumper)
            servoval = 0.6;
        robot.rightslapper.setPosition(servoval);

        servoval = 0.60;
        if (gamepad1.x || gamepad2.left_bumper)
            servoval = 0.05;
        robot.leftslapper.setPosition(servoval);
    }


    public void DriveTrain(){
        //Speed Changing
        boolean rbumper = gamepad1.right_bumper;
        boolean rtrigger = gamepad1.right_trigger > 0.5;
        boolean current = rbumper || rtrigger;
        if (!lastSpeed && current){
            if (rbumper)
                speed++;
            else if (rtrigger)
                speed--;

            if (speed > 2)
                speed = 2;
            if (speed < 0)
                speed = 0;
        }
        lastSpeed = current;

        //Set Motors
        if (runtime.seconds() - accelerateStart < 0.4){
            leftmotorS = speeds[speed] * (runtime.seconds() - accelerateStart) / 0.4;
            rightmotorS = speeds[speed] * (runtime.seconds() - accelerateStart) / 0.4;
        }
        else if (runtime.seconds() - decelerateStart < 0.4){
            leftmotorS = (-1 * speeds[speed] * (runtime.seconds() - decelerateStart) / 0.4) + speeds[speed];
            rightmotorS = (-1 * speeds[speed] * (runtime.seconds() - decelerateStart) / 0.4) + speeds[speed];
        }
        else{
            leftmotorS = speeds[speed];
            rightmotorS = speeds[speed];
        }

        //invert based on stick location
        if (gamepad1.left_stick_y > -gamepad1.left_stick_x)
            leftmotorS *= -1;
        if (gamepad1.left_stick_y > gamepad1.left_stick_x)
            rightmotorS *= -1;

        //make dead zone for stopping
        if (Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1){
            rightmotorS = 0;
            leftmotorS = 0;
        }

        //initialize the accelerator timer start thing
        if (leftmotorS != rightmotorS && accelerateMono) {
            accelerateStart = runtime.seconds();
            accelerateMono = false;
        }
        else if (leftmotorS == 0)
            accelerateMono = true;

        //decelerate mono
        if (decelerateMono && gamepad1.a)
            decelerate = !decelerate;
        else if (!gamepad1.a)
            decelerateMono = true;

        if (decelerate){
            //initialize the decelerator timer start thing
            if (leftmotorS == 0 && decelerateInitMono) {
                decelerateStart = runtime.seconds();
                decelerateInitMono = false;
            }
            else if (leftmotorS != 0)
                decelerateInitMono = true;
        }

        //reverse conversions
        if (gamepad1.b && !reverseMono){
            reverseMono = true;
            reverse = !reverse;
        }
        else if (!gamepad1.b)
            reverseMono = false;
        if (leftmotorS == rightmotorS){
            if (reverse) {
                leftmotorS *= -1;
                rightmotorS *= -1;
            }
        }

        //gyro correct if going fast enough
        if (Math.abs(leftmotorS) > 0.2){
            gyroCorrect();
        }

        //make dead zone for stopping
        if (Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1){
            rightmotorS = 0;
            leftmotorS = 0;
        }

        //assign power
        robot.leftMotor.setPower(leftmotorS);
        robot.rightMotor.setPower(rightmotorS);
    }


    public void Telemetry(){
        //acutal driver feed
        telemetry.addData("Speed", speeds[speed]);
        telemetry.addData("Shot Count", shotCount);
        telemetry.addData("Reverse", reverse);
        telemetry.addData("Decelerate", decelerate);

        //useful
        if (false){
            telemetry.addData("Gyro", robot.gyro.getHeading());
            telemetry.addData("Left", robot.leftcolor.blue());
            telemetry.addData("Right", robot.rightcolor.red());
            telemetry.addData("US1", robot.us1.getUltrasonicLevel());
            telemetry.addData("US2", robot.us2.getUltrasonicLevel());
        }

        //encoders
        if (false){
            telemetry.addData("Encoder Left", robot.leftMotor.getCurrentPosition());
            telemetry.addData("Encoder Right", robot.rightMotor.getCurrentPosition());
            telemetry.addData("Encoder Lift1", robot.cap1.getCurrentPosition());
            telemetry.addData("Encoder Lift2", robot.cap2.getCurrentPosition());
        }

        //deceleration telemetry
        if (false){
            telemetry.addData("speed inputed", (-1 * speeds[speed] * (runtime.seconds() - decelerateStart) / 0.4) + speeds[speed]);
            telemetry.addData("timing", runtime.seconds() - decelerateStart < 0.4);
        }

        //shooting telemetry
        if (false){
            telemetry.addData("Black", black);
            telemetry.addData("Fire", fire);
            telemetry.addData("Cock", cock);
            telemetry.addData("Gate Open", gateOpen);
            telemetry.addData("Shot Count", shotCount);
            telemetry.addData("Extra Spin Reset", extraSpinReset);
            telemetry.addData("Launch Time", launchTime);
            telemetry.addData("FireCommand", fireCommand);
            telemetry.addData("Ready to Fire", readyToFire);
            telemetry.addData("Light", robot.nautilus.getLightDetected());
        }
    }


    public void gyroCorrect(){
        //set gyro now
        double gyronow = runtime.seconds() - gyroLastReset;

        //add reset the gyro durring turns
        if (leftmotorS != rightmotorS) {
            gyroset = true;
            gyroLastReset = runtime.seconds();
        }
        //gyro correct for going straight
        else {
            if (gyroset && gyroLastReset > 0.5) {
                gyrozero = robot.gyro.getHeading();
                gyroset = false;
            }
            if (rightmotorS > 0) {
                if (gyroval > 1 && gyroval < 180)
                    leftmotorS -= 0.2;
                else if (gyroval < -1 || (gyroval > 180 && gyroval < 365))
                    rightmotorS -= 0.2;
            } else {
                if (gyroval > 1 && gyroval < 180)
                    rightmotorS += 0.2;
                else if (gyroval < -1 || (gyroval > 180 && gyroval < 365))
                    leftmotorS += 0.2;
            }

            gyroval = robot.gyro.getHeading() - gyrozero;
        }
    }
}
