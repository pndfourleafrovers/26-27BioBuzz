/*
Turret Servo Tuning OpMode.

The turret rides on TWO standard positional servos ganged on ONE axis (ServoFront + ServoBack).
Because they are physically coupled - and often mounted mirrored - a setPosition() value that
centers one servo will NOT center the other. This OpMode lets you jog each servo live, find the
position where they mechanically agree, and read the exact position/angle values off telemetry so
you can record the turret's starting placement.

The servos are configured for 300 degrees of travel (not 360), so setPosition(0.0..1.0) maps to
0..300 degrees. Telemetry shows both the raw position and the approximate angle.

Controls (gamepad1):
  dpad up / down  - jog the selected servo(s) by the current step (up = +, down = -)
  left bumper     - toggle step size: fine 0.01 (~3 deg) <-> coarse 0.05 (~15 deg)
  x               - select ServoFront only
  b               - select ServoBack only
  y               - select BOTH (jog moves them together, same direction)
  a               - snap both servos to 0.50 (center / known reference)
 */

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "TurretServoTuning", group = "tuning")
public class TurretServoTuning extends LinearOpMode {

    // Which servo(s) the jog controls move.
    private enum Selection { BOTH, FRONT, BACK }

    // Servos are configured for 300 deg of travel, so position 0.0..1.0 => 0..300 deg.
    private static final double SERVO_RANGE_DEGREES = 300.0;

    private static final double FINE_STEP = 0.01;   // ~3 deg
    private static final double COARSE_STEP = 0.05; // ~15 deg
    private static final double CENTER = 0.5;

    private Servo ServoFront = null;
    private Servo ServoBack = null;

    // Commanded positions (start centered). Always clamped to [0.0, 1.0].
    private double posFront = CENTER;
    private double posBack = CENTER;

    private Selection selected = Selection.BOTH;
    private double step = FINE_STEP;

    // Previous-loop button states for edge detection (one tap = one action).
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLb = false;
    private boolean prevX = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevA = false;

    @Override
    public void runOpMode() {

        ServoFront = hardwareMap.get(Servo.class, "ServoFront");
        ServoBack = hardwareMap.get(Servo.class, "ServoBack");

        // Apply the starting (centered) positions before start.
        ServoFront.setPosition(posFront);
        ServoBack.setPosition(posBack);

        telemetry.addData("Status", "Initialized - press PLAY");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ----- Read gamepad with edge detection -----
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean lb = gamepad1.left_bumper;
            boolean x = gamepad1.x;
            boolean b = gamepad1.b;
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;

            // Selection changes
            if (x && !prevX) selected = Selection.FRONT;
            if (b && !prevB) selected = Selection.BACK;
            if (y && !prevY) selected = Selection.BOTH;

            // Step size toggle
            if (lb && !prevLb) {
                step = (step == FINE_STEP) ? COARSE_STEP : FINE_STEP;
            }

            // Center snap (both servos)
            if (a && !prevA) {
                posFront = CENTER;
                posBack = CENTER;
            }

            // Jog the selected servo(s)
            if (up && !prevUp) jogSelected(step);
            if (down && !prevDown) jogSelected(-step);

            // Store states for next loop
            prevUp = up;
            prevDown = down;
            prevLb = lb;
            prevX = x;
            prevB = b;
            prevY = y;
            prevA = a;

            // ----- Apply positions (clamped) -----
            posFront = clamp(posFront);
            posBack = clamp(posBack);
            ServoFront.setPosition(posFront);
            ServoBack.setPosition(posBack);

            // ----- Telemetry -----
            telemetry.addData("Controls", "dpad=jog  LB=step  x=Front b=Back y=Both  a=center");
            telemetry.addData("Selected", selected);
            telemetry.addData("Step", "%.2f  (~%.1f deg)", step, step * SERVO_RANGE_DEGREES);
            telemetry.addLine();
            telemetry.addData("ServoFront", "pos %.3f   (~%.1f deg)", posFront, posFront * SERVO_RANGE_DEGREES);
            telemetry.addData("ServoBack", "pos %.3f   (~%.1f deg)", posBack, posBack * SERVO_RANGE_DEGREES);
            telemetry.update();
        }
    }

    /** Applies a delta to whichever servo(s) are currently selected. */
    private void jogSelected(double delta) {
        switch (selected) {
            case FRONT:
                posFront += delta;
                break;
            case BACK:
                posBack += delta;
                break;
            case BOTH:
            default:
                posFront += delta;
                posBack += delta;
                break;
        }
    }

    /** Clamps a servo position into the valid [0.0, 1.0] range. */
    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
