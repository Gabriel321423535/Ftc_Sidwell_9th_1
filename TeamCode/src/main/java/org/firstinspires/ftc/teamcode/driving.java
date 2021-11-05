/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.teamcode.RobotHardwareOP;
import org.firstinspires.ftc.teamcode.PushbotAutoDriveByEncoder_Linear;




/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driving_2", group="Linear Opmode")

public class driving extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();

    // Encoder things
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.5;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Set servo angle
        int servoAngle = 0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double clawPower;
            boolean servoBool;
            double duckPower;



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Get Claw power
            if (gamepad1.dpad_down) {
                clawPower = 0.0006;
            } else if (gamepad1.dpad_up) {
                clawPower = -0.006;
            } else {
                clawPower = 0;
            }

            if (gamepad1.dpad_up) {
                clawPower = -0.015;
            } else if (gamepad1.dpad_down) {
                clawPower = 0.006;
            } else {
                clawPower = -0.0035;
            }



            // Get servo power
            if (gamepad1.dpad_left)  {
                servoBool = true;
            } else if (gamepad1.dpad_left) {
                servoBool = false;
            }

            // Checking servo Angle
            if (gamepad1.dpad_left) {
                servoAngle = -5000;
            }
            else if (gamepad1.dpad_right) {
                servoAngle = 5000;
            }

            // Setting up duck motor

            if (gamepad1.y) {
                duckPower = 1.0;
            }
            else if (gamepad1.a){
                duckPower = -1.0;
            }
            else {
                duckPower = 0.0;
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //Left power
            robot.back_left.setPower(leftPower);
            robot.front_left.setPower(leftPower);

            //Right power
            robot.back_right.setPower(rightPower);
            robot.front_right.setPower(rightPower);

            //Set Duck motor
            robot.duck_motor.setPower(duckPower);

            //Set claw and servo
            robot.arm_servo.setPosition(servoAngle);

            encoderDrive(DRIVE_SPEED, clawPower, clawPower, 0.1);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Servos", "Servo Power (%2d), right (%.2f)", servoAngle, rightPower);
            telemetry.update();
        }
    }

// Encoder drive
    public void encoderDrive(double speed,
                            double leftInches, double rightInches,
                            double timeoutS) {
            int arm_power_target;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                arm_power_target = robot.arm_motor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                robot.arm_motor.setTargetPosition(arm_power_target);


                // Turn On RUN_TO_POSITION
                robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runtime.reset();
                robot.arm_motor.setPower(Math.abs(speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.arm_motor.isBusy())) {

                    // Display it for the driver.
                    //telemetry.addData("Path1",  "Running to %7d :%7d", arm_power_target);
                    //telemetry.addData("Path2",  "Running at %7d", robot.arm_motor.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.arm_motor.setPower(0);


                // Turn off RUN_TO_POSITION
                robot.arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


                //  sleep(250);   // optional pause after each move
            }

    }
}
