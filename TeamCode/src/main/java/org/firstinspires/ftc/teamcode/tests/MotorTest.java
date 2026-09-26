package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@Autonomous(name="DEBUG: Motor Direction Test", group="Test")
public class MotorTest extends LinearOpMode {

    // Declare motors
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {

        // 1. Initialize Hardware matching your DrivetrainSubsystem names
        leftFront = hardwareMap.get(DcMotorEx.class, "Left_front");
        leftBack  = hardwareMap.get(DcMotorEx.class, "Left_rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "Right_front");
        rightBack  = hardwareMap.get(DcMotorEx.class, "Right_rear");

        // 2. Set Directions (Matches the "Working" configuration we found)
        // Left side = FORWARD, Right side = REVERSE
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // 3. Set Modes
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Motor Test Ready.");
        telemetry.addLine("Press START to cycle through motors.");
        telemetry.addLine("CAUTION: Robot may move!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            
            // --- TEST 1: FRONT LEFT ---
            telemetry.clearAll();
            telemetry.addLine("TESTING: FRONT LEFT");
            telemetry.addLine("Wheel should spin FORWARD");
            telemetry.update();
            
            leftFront.setPower(0.3);
            sleep(2000);
            leftFront.setPower(0);
            sleep(1000);

            // --- TEST 2: FRONT RIGHT ---
            telemetry.clearAll();
            telemetry.addLine("TESTING: FRONT RIGHT");
            telemetry.addLine("Wheel should spin FORWARD");
            telemetry.update();
            
            rightFront.setPower(0.3);
            sleep(2000);
            rightFront.setPower(0);
            sleep(1000);

            // --- TEST 3: BACK LEFT ---
            telemetry.clearAll();
            telemetry.addLine("TESTING: BACK LEFT");
            telemetry.addLine("Wheel should spin FORWARD");
            telemetry.update();
            
            leftBack.setPower(0.3);
            sleep(2000);
            leftBack.setPower(0);
            sleep(1000);

            // --- TEST 4: BACK RIGHT ---
            telemetry.clearAll();
            telemetry.addLine("TESTING: BACK RIGHT");
            telemetry.addLine("Wheel should spin FORWARD");
            telemetry.update();
            
            rightBack.setPower(0.3);
            sleep(2000);
            rightBack.setPower(0);
            sleep(1000);

            // --- DONE ---
            telemetry.clearAll();
            telemetry.addLine("Test Complete.");
            telemetry.update();
            sleep(3000);
        }
    }
}