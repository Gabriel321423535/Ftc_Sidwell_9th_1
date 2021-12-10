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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="a_low_blue_1", group="Autonomous")

public class a_low_blue_1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();

    // Encoder things
    static final double     COUNTS_PER_MOTOR_REV_1    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION_1    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES_1   = 3.55 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_1         = (COUNTS_PER_MOTOR_REV_1 * DRIVE_GEAR_REDUCTION_1) /
            (WHEEL_DIAMETER_INCHES_1 * 3.1415);
    static final double     DRIVE_SPEED_1             = 0.8;
    static final double     TURN_SPEED_1             = 0.2;

    // Encoder things
    static final double     COUNTS_PER_MOTOR_REV_2    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION_2    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES_2   = 0.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH_2         = (COUNTS_PER_MOTOR_REV_2 * DRIVE_GEAR_REDUCTION_2) /
            (WHEEL_DIAMETER_INCHES_2 * 3.1415);
    static final double     DRIVE_SPEED_2             = 0.1;
    static final double     TURN_SPEED_2              = 0.5;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.back_left.getCurrentPosition(),
                robot.back_right.getCurrentPosition(),
                robot.front_left.getCurrentPosition(),
                robot.front_right.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Tighten arm
        robot.arm_servo.setPosition(-10000);
        //Go Towards Tower
        encoderDrive(DRIVE_SPEED_1, 1, 1, 8.0);
        // Lift Arm to top
        encoderDrivearm(DRIVE_SPEED_2, 0.02, 0.02, 1.15);

        robot.arm_motor.setPower(-0.0005);

        encoderDrive(DRIVE_SPEED_1, 3, 3, 8.0);
        sleep(2000);

        // Servo releases
        sleep(2000);
        robot.arm_servo.setPosition(5000);
        sleep(5000);
        // Reverse
        encoderDrive(DRIVE_SPEED_1, -2, -2, 4.0);
        // Put Arm back downs
        encoderDrivearm(DRIVE_SPEED_2, -0.01,-0.01, 0.9);
        robot.arm_motor.setPower(0);



        // Turn to Big bay
        encoderDrive(TURN_SPEED_1, -5, 5, 4.0);

        // Drive forward a bit
        encoderDrive(DRIVE_SPEED_1, 34, 34, 6.0);
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        telemetry.addData("Moved on", "Updating");
        telemetry.update();


    }
    // Encoder drive
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int new_back_LeftTarget;
        int new_front_LeftTarget;
        int new_back_RightTarget;
        int new_front_RightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_back_LeftTarget = robot.back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH_1);
            new_back_RightTarget = robot.back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH_1);
            new_front_LeftTarget = robot.back_left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH_1);
            new_front_RightTarget = robot.back_right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH_1);
            robot.back_left.setTargetPosition(new_back_LeftTarget);
            robot.back_right.setTargetPosition(new_back_RightTarget);
            robot.front_left.setTargetPosition(new_front_LeftTarget);
            robot.front_right.setTargetPosition(new_front_RightTarget);

            // Turn On RUN_TO_POSITION
            robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.back_left.setPower(Math.abs(speed));
            robot.back_right.setPower(Math.abs(speed));
            robot.front_left.setPower(Math.abs(speed));
            robot.front_right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.back_left.isBusy() && robot.back_right.isBusy() && robot.front_left.isBusy() && robot.front_right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", new_back_LeftTarget,  new_front_LeftTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.back_right.getCurrentPosition(),
                        robot.back_right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.back_left.setPower(0);
            robot.back_right.setPower(0);
            robot.front_left.setPower(0);
            robot.front_right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    // Encoder drive
    public void encoderDrivearm(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int arm_power_target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            arm_power_target = robot.arm_motor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH_2);
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
