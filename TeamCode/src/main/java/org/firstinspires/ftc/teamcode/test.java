package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="test")
public class test extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armMotor;
    Servo leftServo;
    Servo rightServo;

    double leftServoOpen = 0.35;
    double rightServoOpen = 0.65;
    double leftServoClose = 0.25;
    double rightServoClose = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("CH_motor1");
        rightMotor = hardwareMap.dcMotor.get("CH_motor0");
        armMotor = hardwareMap.dcMotor.get("CH_motor2");
        leftServo = hardwareMap.get(Servo.class, "CH_servo0");
        rightServo = hardwareMap.get(Servo.class, "CH_servo1");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Autonomous sequence
            leftServo.setPosition(leftServoOpen);
            rightServo.setPosition(rightServoOpen);
            sleep(1000);

            // Drive forward
            leftMotor.setPower(0.25);
            rightMotor.setPower(0.25);
            sleep(2100);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(1000);

            // Close servos
            leftServo.setPosition(leftServoClose);
            rightServo.setPosition(rightServoClose);
            sleep(1000);

            // Move arm
            armMotor.setPower(0.2);
            sleep(1000);
            armMotor.setPower(0);

            // Turn
            int startPos = leftMotor.getCurrentPosition();
            while (leftMotor.getCurrentPosition() > startPos - 2700 && opModeIsActive()) {
                leftMotor.setPower(-0.4);
                rightMotor.setPower(0.4);
                telemetry.addData("Wheel position:", leftMotor.getCurrentPosition());
                telemetry.update();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Drive forward
            leftMotor.setPower(0.25);
            rightMotor.setPower(0.25);
            sleep(2600);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(1000);

            // Extend arm
            armMotor.setPower(0.4);
            sleep(600);
            armMotor.setPower(0);

            // Open servos
            leftServo.setPosition(leftServoOpen);
            rightServo.setPosition(rightServoOpen);
            sleep(1000);

            // Move backward
            leftMotor.setPower(-0.25);
            rightMotor.setPower(-0.25);
            sleep(1800);

            // Turn
            int startPos1 = leftMotor.getCurrentPosition();
            while (leftMotor.getCurrentPosition() < startPos1 + 2700 && opModeIsActive()) {
                leftMotor.setPower(0.4);
                rightMotor.setPower(-0.4);
                telemetry.addData("Wheel position:", leftMotor.getCurrentPosition());
                telemetry.update();
            }
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Drive forward
            leftMotor.setPower(0.25);
            rightMotor.setPower(0.25);
            sleep(2400);

            // Loop repeats
        }
    }
}
