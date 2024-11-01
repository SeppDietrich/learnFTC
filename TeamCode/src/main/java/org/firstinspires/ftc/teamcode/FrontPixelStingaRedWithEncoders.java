package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="FrontPixelStingaRedWithEncoders")

public class FrontPixelStingaRedWithEncoders extends LinearOpMode {

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
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("CH_motor1");
        rightMotor = hardwareMap.dcMotor.get("CH_motor0");

        armMotor = hardwareMap.dcMotor.get("CH_motor2");

        leftServo = hardwareMap.get(Servo.class, "CH_servo0");
        rightServo = hardwareMap.get(Servo.class, "CH_servo1");
        drone= hardwareMap.get(Servo.class, "CH_servo2");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            leftServo.setPosition(leftServoOpen); //90 grade
            rightServo.setPosition(rightServoOpen); //90 grade
            drone.setPosition(0);
            telemetry.update();


            leftMotor.setPower(0.25);
            rightMotor.setPower(0.25);
            sleep(2300);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(1000);
            leftServo.setPosition(leftServoClose);
            rightServo.setPosition(rightServoClose);
            armMotor.setPower(0.2);
            sleep(1000);
            leftMotor.setPower(-0.25);
            rightMotor.setPower(-0.25);
            sleep(100);

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(1000);

            int startPos = leftMotor.getCurrentPosition();
            while (leftMotor.getCurrentPosition() < startPos + 1950) {
                leftMotor.setPower(0.4);
                rightMotor.setPower(-0.4);
                telemetry.addData("Wheell position:", leftMotor.getCurrentPosition());
                telemetry.update();

            }
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            leftMotor.setPower(0.25);
            rightMotor.setPower(0.25);
            armMotor.setPower(0.1);
            sleep(6660);
            armMotor.setPower(0.4);
            sleep(500);
            armMotor.setPower(0);
            leftServo.setPosition(leftServoOpen);
            rightServo.setPosition(rightServoOpen);
//            sleep(1000);
//            leftMotor.setPower(-0.25);
//            rightMotor.setPower(-0.25);
//            sleep(1800);
//            int startPos1 = leftMotor.getCurrentPosition();
//            while (leftMotor.getCurrentPosition() > startPos1 - 2700) {
//                leftMotor.setPower(-0.4);
//                rightMotor.setPower(0.4);
//                telemetry.addData("Wheell position:", leftMotor.getCurrentPosition());
//                telemetry.update();
//
//            }
//            leftMotor.setPower(0.25);
//            rightMotor.setPower(0.25);
//            sleep(1800);
            break;
        }


//        sleep(1000);
//
//        sleep(1000);
//
//        sleep(200);
//
//        sleep(1600);
//        leftMotor.setPower(0.25);
//        rightMotor.setPower(0.25);
//        armMotor.setPower(0.4);
//        sleep(2000);
//        armMotor.setPower(0);
//        leftMotor.setPower(0.25);
//        rightMotor.setPower(0.25);
//
//        sleep(1000);
//        leftServo.setPosition(leftServoOpen);
//        rightServo.setPosition(rightServoOpen);
//        sleep(1000);
//
//        leftMotor.setPower(-0.8);
//        rightMotor.setPower(0.8);
//        sleep(1600);



    }



//    @Override
//    public void loop(){




//        if(gamepad1.left_stick_x>0.05){
//            leftMotor.setPower(gamepad1.left_stick_x);
//            rightMotor.setPower(-gamepad1.left_stick_x);
//        } else if (gamepad1.left_stick_x<-0.05) {
//            leftMotor.setPower(gamepad1.left_stick_x);
//            rightMotor.setPower(-gamepad1.left_stick_x);
//
//        }
//        if(gamepad1.right_trigger>0.05){
//            leftMotor.setPower(gamepad1.right_trigger);
//            rightMotor.setPower(gamepad1.right_trigger);
//        }else if (gamepad1.left_trigger>0.05){
//            leftMotor.setPower(-gamepad1.left_trigger);
//            rightMotor.setPower(-gamepad1.left_trigger);
//        }else {
//            leftMotor.setPower(0);
//            rightMotor.setPower(0);
//        }
//
//        if(gamepad1.dpad_up){
//            armMotor.setPower(0.45);
//
//        } else if (gamepad1.dpad_down) {
//            armMotor.setPower(-0.45);
//
//        }else{
//            armMotor.setPower(0);
//        }
//        if (gamepad1.a && (System.currentTimeMillis() - lastToggleTime > toggleDelay)) {
//            isClose = !isClose;
//            if (isClose) {
//                leftServo.setPosition(leftServoClose);
//                rightServo.setPosition(rightServoClose);
//            } else {
//                leftServo.setPosition(leftServoOpen);
//                rightServo.setPosition(rightServoOpen);
//            }
//            lastToggleTime = System.currentTimeMillis();  // Update last toggle time
//        }



//        telemetry.addData("Time", getRuntime());
//        telemetry.update();

    //}
}
