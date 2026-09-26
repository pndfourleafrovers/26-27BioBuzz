package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.PinpointDrive;

@TeleOp(name="Drive Tuning All Axes", group="Tuning")
@Disabled
public class OdoPIDFTuning extends OpMode {

    PinpointDrive drive;
    GoBildaPinpointDriver pinpoint; // Need this to read current position

    // Tuning State
    enum Mode { TUNING_DRIVE, TUNING_TURN }
    Mode currentMode = Mode.TUNING_DRIVE;

    double[] stepSizes = {0.001, 0.0001, 0.00001};
    int stepIndex = 1;

    // Target Positions
    double targetX = 0;
    double targetY = 0;
    double targetH = 0;

    // Toggle Logic
    boolean lastStart = false;
    boolean lastSelect = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastLeft = false;
    boolean lastRight = false;

    @Override
    public void init() {
        drive = new PinpointDrive(hardwareMap);

        // Initialize Pinpoint just like in Auto to read position
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.setOffsets(-70.0, -360.0, DistanceUnit.MM); // Check your offsets!
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("READY: Press START to switch Drive/Turn Mode");
    }

    @Override
    public void loop() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();

        // ------------------------------------------------
        // 1. SWITCH TUNING MODE (Start Button)
        // ------------------------------------------------

        if (gamepad1.start && !lastStart) {
            if (currentMode == Mode.TUNING_DRIVE) {
                currentMode = Mode.TUNING_TURN;
            } else {
                currentMode = Mode.TUNING_DRIVE;
            }
        }
        lastStart = gamepad1.start;

        // Cycle Step Size (Select/Back Button)
        if (gamepad1.back && !lastSelect) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastSelect = gamepad1.back;


        // ------------------------------------------------
        // 2. ADJUST CONSTANTS (D-PAD)
        // ------------------------------------------------

        if (currentMode == Mode.TUNING_DRIVE) {
            // Adjust DRIVE_P / DRIVE_D
            if (gamepad1.dpad_up && !lastUp)     PinpointDrive.DRIVE_P += stepSizes[stepIndex];
            if (gamepad1.dpad_down && !lastDown) PinpointDrive.DRIVE_P -= stepSizes[stepIndex];
            if (gamepad1.dpad_right && !lastRight) PinpointDrive.DRIVE_D += stepSizes[stepIndex];
            if (gamepad1.dpad_left && !lastLeft)   PinpointDrive.DRIVE_D -= stepSizes[stepIndex];
        } else {
            // Adjust TURN_P / TURN_D
            if (gamepad1.dpad_up && !lastUp)     PinpointDrive.TURN_P += stepSizes[stepIndex];
            if (gamepad1.dpad_down && !lastDown) PinpointDrive.TURN_P -= stepSizes[stepIndex];
            if (gamepad1.dpad_right && !lastRight) PinpointDrive.TURN_D += stepSizes[stepIndex];
            if (gamepad1.dpad_left && !lastLeft)   PinpointDrive.TURN_D -= stepSizes[stepIndex];
        }
        // Update latches
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;


        // ------------------------------------------------
        // 3. SET TEST TARGETS (Action Buttons)
        // ------------------------------------------------

        // A = Go Out (Forward 1200mm)
        if (gamepad1.a) {
            targetX = 1200; targetY = 0; targetH = 0;
        }
        // B = Go Home (0, 0, 0)
        if (gamepad1.b) {
            targetX = 0; targetY = 0; targetH = 0;
        }

        // X = Strafe Left (Y = 600mm)
        if (gamepad1.x) {
            targetX = 0; targetY = 600; targetH = 0;
        }
        // Y = Strafe Right (Y = -600mm)
        if (gamepad1.y) {
            targetX = 0; targetY = -600; targetH = 0;
        }

        // Left Bumper = Turn 90 Deg Left
        if (gamepad1.left_bumper) {
            targetX = 0; targetY = 0; targetH = 90;
        }
        // Right Bumper = Turn 90 Deg Right
        if (gamepad1.right_bumper) {
            targetX = 0; targetY = 0; targetH = -90;
        }


        // ------------------------------------------------
        // 4. RUN LOOP
        // ------------------------------------------------

        // Pass the live position from Pinpoint to the Drive Class
        drive.runToPosition(
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.RADIANS),
                targetX,
                targetY,
                targetH,
                0.7, // Max Drive Speed
                0.5  // Max Turn Speed
        );


        // ------------------------------------------------
        // 5. TELEMETRY
        // ------------------------------------------------
        telemetry.addData("MODE", currentMode);
        telemetry.addData("Step Size", "%.5f (Back Btn)", stepSizes[stepIndex]);
        telemetry.addLine("--------------------------");

        if (currentMode == Mode.TUNING_DRIVE) {
            telemetry.addData("DRIVE P", "%.5f", PinpointDrive.DRIVE_P);
            telemetry.addData("DRIVE D", "%.5f", PinpointDrive.DRIVE_D);
        } else {
            telemetry.addData("TURN P", "%.5f", PinpointDrive.TURN_P);
            telemetry.addData("TURN D", "%.5f", PinpointDrive.TURN_D);
        }

        telemetry.addLine("--------------------------");
        telemetry.addData("Target", "X:%.0f Y:%.0f H:%.0f", targetX, targetY, targetH);
        telemetry.addData("Current", "X:%.0f Y:%.0f H:%.0f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), Math.toDegrees(pos.getHeading(AngleUnit.RADIANS)));

        telemetry.update();
    }
}