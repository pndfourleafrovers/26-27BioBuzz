package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * DIAGNOSTIC: verifies all four drive encoders are actually counting.
 *
 * Why this matters: DrivetrainSubsystem.runDriveToPosition ends a move when NO motor is still busy
 * (FL.isBusy() || RL.isBusy() || FR.isBusy() || RR.isBusy()). A motor whose encoder never counts stays
 * "busy" forever, so the move can only end on the 3-second timeout. That produces two symptoms at once:
 * commanded distances get ignored (every move runs the full timeout), and the robot veers (the motors
 * with working encoders stop at target while the dead-encoder ones keep driving).
 *
 * HOW TO USE (no power is applied to the motors - the robot will not drive itself):
 *   1. Run this op mode and press start.
 *   2. PUSH THE ROBOT FORWARD BY HAND a few feet.
 *   3. Watch the four counts. All four should change by a similar amount and in the same direction.
 *        - A count stuck at 0 = that encoder is not wired / not working. Fix it before trusting auto.
 *        - A count moving the opposite way = that motor's direction or encoder is reversed.
 *   4. Press A to zero the counts and repeat.
 *
 * The counts here are RAW (motor direction is applied, matching DrivetrainSubsystem's setup), so forward
 * pushing should move all four in the SAME direction.
 */
@TeleOp(name = "DEBUG: Drive Encoder Check", group = "Test")
public class EncoderCheck extends LinearOpMode {

    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        // Same config names and directions as DrivetrainSubsystem, so readings match what auto sees.
        leftFront  = hardwareMap.get(DcMotorEx.class, "Left_front");
        leftBack   = hardwareMap.get(DcMotorEx.class, "Left_rear");
        rightFront = hardwareMap.get(DcMotorEx.class, "Right_front");
        rightBack  = hardwareMap.get(DcMotorEx.class, "Right_rear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        telemetry.addLine("Encoder check ready. Press START, then PUSH THE ROBOT FORWARD by hand.");
        telemetry.addLine("No motor power is applied - the robot will not move on its own.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                resetEncoders();
            }

            int lf = leftFront.getCurrentPosition();
            int lb = leftBack.getCurrentPosition();
            int rf = rightFront.getCurrentPosition();
            int rb = rightBack.getCurrentPosition();

            telemetry.addLine("Push the robot FORWARD - all four should count together.");
            telemetry.addLine("A = zero the counts");
            telemetry.addLine();
            telemetry.addData("Left_front ", "%8d %s", lf, flag(lf));
            telemetry.addData("Left_rear  ", "%8d %s", lb, flag(lb));
            telemetry.addData("Right_front", "%8d %s", rf, flag(rf));
            telemetry.addData("Right_rear ", "%8d %s", rb, flag(rb));
            telemetry.addLine();
            telemetry.addData("Spread", "%d ticks (large spread = wheels slipping or an encoder is off)",
                    Math.max(Math.max(lf, lb), Math.max(rf, rb)) - Math.min(Math.min(lf, lb), Math.min(rf, rb)));
            telemetry.update();
        }
    }

    /** Flags a count that hasn't moved at all - the signature of a disconnected/dead encoder. */
    private String flag(int count) {
        return (count == 0) ? "  <-- NOT COUNTING?" : "";
    }

    private void resetEncoders() {
        DcMotorEx[] motors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // RUN_WITHOUT_ENCODER so the encoders still report position but nothing drives the motors.
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }
    }
}
