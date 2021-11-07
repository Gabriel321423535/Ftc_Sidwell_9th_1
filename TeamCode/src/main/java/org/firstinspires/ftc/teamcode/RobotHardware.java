package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class RobotHardware {
    //Instantiate motors and servos
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor arm_motor;
    public DcMotor duck_motor;
    public Servo arm_servo;
    public DigitalChannel digitalTouch;


    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        duck_motor = hardwareMap.get(DcMotor.class, "duck_motor");

        // Set Motor Power
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
        arm_motor.setPower(0);
        arm_servo.setPosition(180);
        duck_motor.setPower(0);


        //Something


        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        //Set Motor Mode
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duck_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set Direction
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        arm_motor.setDirection(DcMotor.Direction.FORWARD);
        arm_servo.setDirection(Servo.Direction.FORWARD);
        duck_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Set Zero power mode
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}