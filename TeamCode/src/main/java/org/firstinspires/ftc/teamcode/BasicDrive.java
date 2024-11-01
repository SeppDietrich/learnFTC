package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="BasicDrive")
public class BasicDrive extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    DcMotor armMotor;

    Servo leftServo;
    Servo rightServo;
    Servo drone;
    double leftServoOpen = 0.35;
    double rightServoOpen =0.65;
    double leftServoClose = 0.25;
    double rightServoClose =0.75;
    boolean isClose = false;
    long lastToggleTime = 0;  // Variable to store the time of the last toggle
    long toggleDelay = 1000;
    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("CH_motor1");
        rightMotor = hardwareMap.dcMotor.get("CH_motor0");

        armMotor = hardwareMap.dcMotor.get("CH_motor2");

        leftServo = hardwareMap.get(Servo.class, "CH_servo0");
        rightServo = hardwareMap.get(Servo.class, "CH_servo1");
        drone= hardwareMap.get(Servo.class, "CH_servo2");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo.setPosition(leftServoOpen); //90 grade
        rightServo.setPosition(rightServoOpen); //90 grade
        drone.setPosition(0);
        telemetry.update();

    }

    @Override
    public void loop(){
        if(gamepad1.left_stick_x>0.05){
            leftMotor.setPower(gamepad1.left_stick_x);
            rightMotor.setPower(-gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_x<-0.05) {
            leftMotor.setPower(gamepad1.left_stick_x);
            rightMotor.setPower(-gamepad1.left_stick_x);

        }
       if(gamepad1.right_trigger>0.05){
           leftMotor.setPower(gamepad1.right_trigger);
           rightMotor.setPower(gamepad1.right_trigger);
       }else if (gamepad1.left_trigger>0.05){
           leftMotor.setPower(-gamepad1.left_trigger);
           rightMotor.setPower(-gamepad1.left_trigger);
        }else {
           leftMotor.setPower(0);
           rightMotor.setPower(0);
       }

        if(gamepad1.dpad_up){
            armMotor.setPower(0.45);

        } else if (gamepad1.dpad_down) {
            armMotor.setPower(-0.45);

        }else{
            armMotor.setPower(0);
        }
        if (gamepad1.a && (System.currentTimeMillis() - lastToggleTime > toggleDelay)) {
            isClose = !isClose;
            if (isClose) {
                leftServo.setPosition(leftServoClose);
                rightServo.setPosition(rightServoClose);
            } else {
                leftServo.setPosition(leftServoOpen);
                rightServo.setPosition(rightServoOpen);
            }
            lastToggleTime = System.currentTimeMillis();  // Update last toggle time
        }
        if (gamepad1.square){
            drone.setPosition(0.5);
        }



        telemetry.addData("Time", getRuntime());
        telemetry.update();

    }
}