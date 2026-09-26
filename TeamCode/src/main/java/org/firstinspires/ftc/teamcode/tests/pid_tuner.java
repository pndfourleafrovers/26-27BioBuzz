package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.PinpointDrive;

/**
 * PID Tuning Tool for Pinpoint Drive
 * 
 * HOW TO USE:
 * 1. Select which axis to tune (Drive, Strafe, or Turn)
 * 2. Run automated tests to target positions
 * 3. Adjust P and D values based on behavior
 * 4. Copy final values back to PinpointDrive.java
 * 
 * CONTROLS:
 * D-PAD UP/DOWN: Select tuning mode (Drive/Strafe/Turn)
 * A: Start automated test
 * B: Stop test
 * X: Reset position
 * Y: Toggle between P and D adjustment
 * 
 * LEFT STICK Y: Adjust selected value (up=increase, down=decrease)
 * LEFT BUMPER: Fine adjustment mode (smaller increments)
 * RIGHT BUMPER: Coarse adjustment mode (larger increments)
 * 
 * START: Save current values to telemetry log
 * BACK: Reset to default values
 */
@Disabled
@TeleOp(name="PID Tuner", group="Test")
public class pid_tuner extends OpMode {

    private PinpointDrive autoDrive;
    private GoBildaPinpointDriver pinpoint;

    // Odometry configuration
    private final double X_POD_OFFSET = -38.1;
    private final double Y_POD_OFFSET = -152.4;

    // Starting position
    private final double START_X = 0.0;
    private final double START_Y = 0.0;
    private final double START_H = 0.0;

    // Test targets
    private double targetX = 0;
    private double targetY = 0;
    private double targetH = 0;

    // Current PID values being tuned
    private double currentDriveP;
    private double currentDriveD;
    private double currentStrafeP;
    private double currentStrafeD;
    private double currentTurnP;
    private double currentTurnD;

    // Default values from your PinpointDrive.java
    private final double DEFAULT_DRIVE_P = 0.00071;
    private final double DEFAULT_DRIVE_D = 0.00008;
    private final double DEFAULT_STRAFE_P = 0.0012;
    private final double DEFAULT_STRAFE_D = 0.0001;
    private final double DEFAULT_TURN_P = 0.00054;
    private final double DEFAULT_TURN_D = 0.00023;

    // Tuning state
    private enum TuningMode {
        DRIVE,
        STRAFE,
        TURN
    }
    private TuningMode tuningMode = TuningMode.DRIVE;

    private enum AdjustmentTarget {
        P_VALUE,
        D_VALUE
    }
    private AdjustmentTarget adjustmentTarget = AdjustmentTarget.P_VALUE;

    private enum AdjustmentSpeed {
        FINE,
        NORMAL,
        COARSE
    }
    private AdjustmentSpeed adjustmentSpeed = AdjustmentSpeed.NORMAL;

    // Test state
    private boolean isRunningTest = false;
    private double testStartTime = 0;
    private double testStartX = 0;
    private double testStartY = 0;
    private double testStartH = 0;

    // Performance tracking
    private double maxOvershoot = 0;
    private double settlingTime = 0;
    private double steadyStateError = 0;
    private boolean hasSettled = false;

    // Button debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastStart = false;
    private boolean lastBack = false;

    @Override
    public void init() {
        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        try {
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addData("Warning", "IMU init failed");
        }

        pinpoint.setPosX(START_X, DistanceUnit.MM);
        pinpoint.setPosY(START_Y, DistanceUnit.MM);
        pinpoint.setHeading(START_H, AngleUnit.DEGREES);

        // Initialize drive
        autoDrive = new PinpointDrive(hardwareMap);

        // Set initial values from defaults
        currentDriveP = DEFAULT_DRIVE_P;
        currentDriveD = DEFAULT_DRIVE_D;
        currentStrafeP = DEFAULT_STRAFE_P;
        currentStrafeD = DEFAULT_STRAFE_D;
        currentTurnP = DEFAULT_TURN_P;
        currentTurnD = DEFAULT_TURN_D;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Ready to tune PID");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.update();

        double curX = pinpoint.getPosX(DistanceUnit.MM);
        double curY = pinpoint.getPosY(DistanceUnit.MM);
        double curH = pinpoint.getHeading(AngleUnit.DEGREES);

        // Handle button presses
        handleButtons(curX, curY, curH);

        // Handle analog adjustments
        handleAnalogAdjustments();

        // Update PID values in the drive class
        updateDrivePIDValues();

        // Run test if active
        if (isRunningTest) {
            runTest(curX, curY, curH);
        }

        // Display telemetry
        displayTelemetry(curX, curY, curH);
    }

    private void handleButtons(double curX, double curY, double curH) {
        // Mode selection
        if (gamepad1.dpad_up && !lastDpadUp) {
            cycleTuningModeUp();
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            cycleTuningModeDown();
        }

        // Test control
        if (gamepad1.a && !lastA && !isRunningTest) {
            startTest(curX, curY, curH);
        }
        if (gamepad1.b && !lastB) {
            stopTest();
        }

        // Position reset
        if (gamepad1.x && !lastX) {
            resetPosition();
        }

        // Toggle P/D adjustment
        if (gamepad1.y && !lastY) {
            adjustmentTarget = (adjustmentTarget == AdjustmentTarget.P_VALUE) 
                ? AdjustmentTarget.D_VALUE 
                : AdjustmentTarget.P_VALUE;
        }

        // Save values
        if (gamepad1.start && !lastStart) {
            saveCurrentValues();
        }

        // Reset to defaults
        if (gamepad1.back && !lastBack) {
            resetToDefaults();
        }

        // Update button states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastStart = gamepad1.start;
        lastBack = gamepad1.back;
    }

    private void handleAnalogAdjustments() {
        // Determine adjustment speed
        if (gamepad1.left_bumper) {
            adjustmentSpeed = AdjustmentSpeed.FINE;
        } else if (gamepad1.right_bumper) {
            adjustmentSpeed = AdjustmentSpeed.COARSE;
        } else {
            adjustmentSpeed = AdjustmentSpeed.NORMAL;
        }

        // Get stick input
        float stickY = -gamepad1.left_stick_y; // Inverted because up is negative

        if (Math.abs(stickY) > 0.1) {
            adjustValue(stickY);
        }
    }

    private void adjustValue(float direction) {
        double increment;
        
        // Determine increment size based on mode and speed
        switch (adjustmentSpeed) {
            case FINE:
                increment = 0.00001 * direction;
                break;
            case COARSE:
                increment = 0.0001 * direction;
                break;
            case NORMAL:
            default:
                increment = 0.00005 * direction;
                break;
        }

        // Apply to appropriate value
        switch (tuningMode) {
            case DRIVE:
                if (adjustmentTarget == AdjustmentTarget.P_VALUE) {
                    currentDriveP = Math.max(0, currentDriveP + increment);
                } else {
                    currentDriveD = Math.max(0, currentDriveD + increment);
                }
                break;
            case STRAFE:
                if (adjustmentTarget == AdjustmentTarget.P_VALUE) {
                    currentStrafeP = Math.max(0, currentStrafeP + increment);
                } else {
                    currentStrafeD = Math.max(0, currentStrafeD + increment);
                }
                break;
            case TURN:
                if (adjustmentTarget == AdjustmentTarget.P_VALUE) {
                    currentTurnP = Math.max(0, currentTurnP + increment);
                } else {
                    currentTurnD = Math.max(0, currentTurnD + increment);
                }
                break;
        }
    }

    private void updateDrivePIDValues() {
        PinpointDrive.DRIVE_P = currentDriveP;
        PinpointDrive.DRIVE_D = currentDriveD;
        PinpointDrive.STRAFE_P = currentStrafeP;
        PinpointDrive.STRAFE_D = currentStrafeD;
        PinpointDrive.TURN_P = currentTurnP;
        PinpointDrive.TURN_D = currentTurnD;
    }

    private void startTest(double curX, double curY, double curH) {
        isRunningTest = true;
        testStartTime = getRuntime();
        testStartX = curX;
        testStartY = curY;
        testStartH = curH;
        maxOvershoot = 0;
        settlingTime = 0;
        hasSettled = false;

        // Set target based on mode
        switch (tuningMode) {
            case DRIVE:
                targetX = curX;
                targetY = curY + 1000; // Drive forward 1000mm
                targetH = curH;
                break;
            case STRAFE:
                targetX = curX + 1000; // Strafe right 1000mm
                targetY = curY;
                targetH = curH;
                break;
            case TURN:
                targetX = curX;
                targetY = curY;
                targetH = curH + 90; // Turn 90 degrees
                break;
        }
    }

    private void runTest(double curX, double curY, double curH) {
        boolean atTarget = autoDrive.runToPosition(
            curX, curY, curH,
            targetX, targetY, targetH,
            0.6, 0.3
        );

        double elapsedTime = getRuntime() - testStartTime;

        // Calculate error based on mode
        double error = 0;
        switch (tuningMode) {
            case DRIVE:
                error = Math.abs(targetY - curY);
                break;
            case STRAFE:
                error = Math.abs(targetX - curX);
                break;
            case TURN:
                double headingError = targetH - curH;
                while (headingError > 180) headingError -= 360;
                while (headingError <= -180) headingError += 360;
                error = Math.abs(headingError);
                break;
        }

        // Track overshoot
        if (error > maxOvershoot) {
            maxOvershoot = error;
        }

        // Check if settled (error < tolerance for 0.5 seconds)
        if (error < 10 && !hasSettled) {
            if (settlingTime == 0) {
                settlingTime = elapsedTime;
            } else if (elapsedTime - settlingTime > 0.5) {
                hasSettled = true;
            }
        } else if (error >= 10) {
            settlingTime = 0;
        }

        // Auto-stop after reaching target or timeout
        if (atTarget || elapsedTime > 5.0) {
            steadyStateError = error;
            stopTest();
        }
    }

    private void stopTest() {
        isRunningTest = false;
        autoDrive.stop();
    }

    private void resetPosition() {
        pinpoint.setPosX(START_X, DistanceUnit.MM);
        pinpoint.setPosY(START_Y, DistanceUnit.MM);
        pinpoint.setHeading(START_H, AngleUnit.DEGREES);
        pinpoint.update();
        stopTest();
    }

    private void cycleTuningModeUp() {
        switch (tuningMode) {
            case DRIVE:
                tuningMode = TuningMode.STRAFE;
                break;
            case STRAFE:
                tuningMode = TuningMode.TURN;
                break;
            case TURN:
                tuningMode = TuningMode.DRIVE;
                break;
        }
    }

    private void cycleTuningModeDown() {
        switch (tuningMode) {
            case DRIVE:
                tuningMode = TuningMode.TURN;
                break;
            case STRAFE:
                tuningMode = TuningMode.DRIVE;
                break;
            case TURN:
                tuningMode = TuningMode.STRAFE;
                break;
        }
    }

    private void saveCurrentValues() {
        telemetry.log().add("═══ SAVED VALUES ═══");
        telemetry.log().add(String.format("DRIVE_P = %.5f", currentDriveP));
        telemetry.log().add(String.format("DRIVE_D = %.5f", currentDriveD));
        telemetry.log().add(String.format("STRAFE_P = %.5f", currentStrafeP));
        telemetry.log().add(String.format("STRAFE_D = %.5f", currentStrafeD));
        telemetry.log().add(String.format("TURN_P = %.5f", currentTurnP));
        telemetry.log().add(String.format("TURN_D = %.5f", currentTurnD));
        telemetry.log().add("═══════════════════");
    }

    private void resetToDefaults() {
        currentDriveP = DEFAULT_DRIVE_P;
        currentDriveD = DEFAULT_DRIVE_D;
        currentStrafeP = DEFAULT_STRAFE_P;
        currentStrafeD = DEFAULT_STRAFE_D;
        currentTurnP = DEFAULT_TURN_P;
        currentTurnD = DEFAULT_TURN_D;
    }

    private void displayTelemetry(double curX, double curY, double curH) {
        telemetry.addData("═══ PID TUNER ═══", "");
        telemetry.addData("Mode", tuningMode);
        telemetry.addData("Adjusting", adjustmentTarget);
        telemetry.addData("Speed", adjustmentSpeed);
        telemetry.addData("", "");

        // Show current values for selected mode
        switch (tuningMode) {
            case DRIVE:
                telemetry.addData("DRIVE_P", "%.5f %s", currentDriveP, 
                    adjustmentTarget == AdjustmentTarget.P_VALUE ? "◄" : "");
                telemetry.addData("DRIVE_D", "%.5f %s", currentDriveD,
                    adjustmentTarget == AdjustmentTarget.D_VALUE ? "◄" : "");
                break;
            case STRAFE:
                telemetry.addData("STRAFE_P", "%.5f %s", currentStrafeP,
                    adjustmentTarget == AdjustmentTarget.P_VALUE ? "◄" : "");
                telemetry.addData("STRAFE_D", "%.5f %s", currentStrafeD,
                    adjustmentTarget == AdjustmentTarget.D_VALUE ? "◄" : "");
                break;
            case TURN:
                telemetry.addData("TURN_P", "%.5f %s", currentTurnP,
                    adjustmentTarget == AdjustmentTarget.P_VALUE ? "◄" : "");
                telemetry.addData("TURN_D", "%.5f %s", currentTurnD,
                    adjustmentTarget == AdjustmentTarget.D_VALUE ? "◄" : "");
                break;
        }

        telemetry.addData("", "");
        telemetry.addData("Position", "");
        telemetry.addData("  X", "%.0f mm", curX);
        telemetry.addData("  Y", "%.0f mm", curY);
        telemetry.addData("  H", "%.1f°", curH);

        if (isRunningTest) {
            telemetry.addData("", "");
            telemetry.addData("⚙ TEST RUNNING", "");
            telemetry.addData("Target", "");
            switch (tuningMode) {
                case DRIVE:
                    telemetry.addData("  Y", "%.0f mm", targetY);
                    telemetry.addData("Error", "%.0f mm", Math.abs(targetY - curY));
                    break;
                case STRAFE:
                    telemetry.addData("  X", "%.0f mm", targetX);
                    telemetry.addData("Error", "%.0f mm", Math.abs(targetX - curX));
                    break;
                case TURN:
                    telemetry.addData("  H", "%.1f°", targetH);
                    double hErr = targetH - curH;
                    while (hErr > 180) hErr -= 360;
                    while (hErr <= -180) hErr += 360;
                    telemetry.addData("Error", "%.1f°", Math.abs(hErr));
                    break;
            }
        } else {
            telemetry.addData("", "");
            telemetry.addData("Performance", "");
            telemetry.addData("  Max Overshoot", "%.1f", maxOvershoot);
            telemetry.addData("  Settling Time", "%.2f s", settlingTime);
            telemetry.addData("  Steady Error", "%.1f", steadyStateError);
        }

        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  D-Pad ↕", "Change mode");
        telemetry.addData("  Y", "Toggle P/D");
        telemetry.addData("  L-Stick ↕", "Adjust value");
        telemetry.addData("  L/R Bumper", "Fine/Coarse");
        telemetry.addData("  A", "Run test");
        telemetry.addData("  B", "Stop test");
        telemetry.addData("  X", "Reset position");
        telemetry.addData("  START", "Save to log");
        telemetry.addData("  BACK", "Reset defaults");

        telemetry.update();
    }
}