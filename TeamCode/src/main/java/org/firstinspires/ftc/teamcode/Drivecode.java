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
 */


@TeleOp(name="Drive: Drivecode", group="Drive")
//@Disabled

public class Drivecode extends LinearOpMode {


    RobotConifg         robot     = new RobotConifg();
    private ElapsedTime runtime   = new ElapsedTime();

    double harvesterspeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //DRIVE CONTROLS
        double[] speeds = {0.5, 0.75, 1.0};
        int speed = 1;

        boolean lastSpeed = false;

        boolean gyroset = false;
        double gyrozero = 0;

        double gyroval = 0;

        robot.init(hardwareMap);

        telemetry.addData("Jarvis", "Good luck sir!");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            //ACCELERATOR SECTION----------------------------------------------------

            double Aspeed =0;
            if(gamepad2.y)
                Aspeed= 1;
            else if (gamepad2.b)
                Aspeed= .75;
            else if (gamepad2.a)
                Aspeed= .50;
            if (gamepad2.right_bumper)
                Aspeed *=-1;
            robot.accelerator.setPower(Aspeed);


            //END ACCELERATOR SECTION------------------------------------------------

            //HARVESTER SECTION------------------------------------------------------
            //speed declaration
            if (gamepad2.dpad_up)
                harvesterspeed = -0.75;
            else if (gamepad2.dpad_down)
                harvesterspeed = 0.75;
            else if (gamepad2.dpad_left)
                harvesterspeed = 0;

            robot.harvester.setPower(harvesterspeed);

            //END HARVESTER SECTION--------------------------------------------------

            //WINGS SECTION----------------------------------------------------------

            double servoval = 0.05;
            if (gamepad1.y)
                servoval = 0.6;
            robot.rightslapper.setPosition(servoval);

            servoval = 0.6;
            if (gamepad1.x)
                servoval = 0.05;
            robot.leftslapper.setPosition(servoval);

            //END WINGS SECTION------------------------------------------------------

            //MOTOR SECTION ---------------------------------------------------------

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
            double leftmotorS = speeds[speed];
            double rightmotorS = speeds[speed];

            //invert based on stick location
            if (gamepad1.left_stick_y > gamepad1.left_stick_x)
                leftmotorS *= -1;
            if (gamepad1.left_stick_y > -gamepad1.left_stick_x)
                rightmotorS *= -1;


            //make dead zone for stopping
            if (Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1){
                rightmotorS = 0;
                leftmotorS = 0;
            }
            //add reset the gyro durring turns
            else if (leftmotorS != rightmotorS) {
                gyroset = true;
                runtime.reset();
            }
            //gyro correct for going straight
            else{
                if (gyroset && runtime.milliseconds() > 500){
                    gyrozero = robot.gyro.getHeading();
                    gyroset = false;
                }
                if (rightmotorS > 0) {
                    if (gyroval > 1 && gyroval < 180)
                        leftmotorS -= 0.2;
                    else if (gyroval < -1 || (gyroval > 180 && gyroval < 365))
                        rightmotorS -= 0.2;
                }
                else {
                    if (gyroval > 1 && gyroval < 180)
                        rightmotorS -= 0.2;
                    else if (gyroval < -1 || (gyroval > 180 && gyroval < 365))
                        leftmotorS -= 0.2;
                }

                gyroval = robot.gyro.getHeading() - gyrozero;
            }

            //assign power
            robot.leftMotor.setPower(leftmotorS);
            robot.rightMotor.setPower(rightmotorS);

            //END MOTOR SECTION------------------------------------------------------

            //Update Drive Train Output Data
            //acutal driver feed
            telemetry.addData("Speed", speeds[speed]);
            telemetry.addData("Left", robot.leftcolor.red());
            telemetry.addData("Right", robot.rightcolor.red());

            //CLOSING INFORMATION
            telemetry.update();
            robot.waitForTick(40);
            idle();
        }
    }
}
