package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;

/**
 * Enhanced Odometry Diagnostic Tool with IMU Testing
 * 
 * INSTRUCTIONS:
 * 1. Place robot at a known starting position
 * 2. Check IMU status on init screen
 * 3. Use D-pad to run each test individually
 * 4. Measure actual distance traveled with a tape measure
 * 5. Compare measured distance to telemetry readings
 * 
 * CONTROLS:
 * - D-Pad UP: Drive forward 500mm
 * - D-Pad DOWN: Drive backward 500mm
 * - D-Pad LEFT: Strafe left 500mm
 * - D-Pad RIGHT: Strafe right 500mm
 * - X: Rotate clockwise 90°
 * - B: Rotate counter-clockwise 90°
 * - LEFT BUMPER: Pure rotation test (360° spin)
 * - RIGHT BUMPER: Figure-8 pattern test
 * - A: Reset position to (0, 0, 0°)
 * - Y: Cycle through display modes
 * - START: Save current position as reference
 * - BACK: Return to saved reference position
 */
@Disabled
@TeleOp(name="Enhanced Odometry Diagnostic", group="Test")
public class odometry_diagnostic extends OpMode {

    private DcMotorEx FrontLeft, RearLeft, FrontRight, RearRight;
    private GoBildaPinpointDriver pinpoint;

    // Odometry configuration - MATCH YOUR WorkingBlueFar.java VALUES
    private final double X_POD_OFFSET = 60.0;
    private final double Y_POD_OFFSET = -184.0;

    // Test parameters
    private final double TEST_DISTANCE = 500.0; // mm
    private final double TEST_ANGLE = 90.0; // degrees
    private final double TEST_SPEED = 0.3;
    private final double ROTATION_TEST_SPEED = 0.25;

    private ElapsedTime testTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isTesting = false;
    private String currentTest = "None";

    // Store starting position for each test
    private double testStartX = 0;
    private double testStartY = 0;
    private double testStartH = 0;

    // Reference position storage
    private double refX = 0;
    private double refY = 0;
    private double refH = 0;
    private boolean hasReference = false;

    // Button press detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastStart = false;
    private boolean lastBack = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // Display modes
    private enum DisplayMode {
        BASIC,          // Position and controls
        DETAILED,       // Add velocities and loop times
        ENCODER_RAW,    // Show raw encoder values
        IMU_STATUS      // Show IMU diagnostics
    }

    private DisplayMode displayMode = DisplayMode.BASIC;

    // IMU diagnostics
    private boolean imuInitialized = false;
    private String imuStatus = "Unknown";

    // Performance tracking
    private double lastLoopTime = 0;
    private double avgLoopTime = 0;
    private int loopCount = 0;

    // Complex test state
    private enum ComplexTest {
        NONE,
        ROTATION_360,
        FIGURE_8
    }

    private ComplexTest complexTest = ComplexTest.NONE;
    private int testPhase = 0;

    @Override
    public void init() {
        // Initialize motors
        FrontLeft = hardwareMap.get(DcMotorEx.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotorEx.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotorEx.class, "Right_front");
        RearRight = hardwareMap.get(DcMotorEx.class, "Right_rear");

        // Set directions - MATCH YOUR PinpointDrive.java
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Pinpoint
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
            pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // MATCH YOUR WorkingBlueFar.java ENCODER DIRECTIONS
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );

            // Try to initialize IMU
            try {
                pinpoint.resetPosAndIMU();
                imuInitialized = true;
                imuStatus = "IMU Initialized ✓";
            } catch (Exception e) {
                imuInitialized = false;
                imuStatus = "IMU Failed: " + e.getMessage();
            }

            // Start at origin for testing
            pinpoint.setPosX(0, DistanceUnit.MM);
            pinpoint.setPosY(0, DistanceUnit.MM);
            pinpoint.setHeading(0, AngleUnit.DEGREES);

        } catch (Exception e) {
            imuStatus = "Pinpoint Init Failed: " + e.getMessage();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("IMU Status", imuStatus);
        telemetry.addData("", "Ready for diagnostic tests");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        Pose2D pos = pinpoint.getPosition();

        telemetry.addData("═══ INITIALIZATION ═══", "");
        telemetry.addData("IMU Status", imuStatus);
        telemetry.addData("Using IMU for Heading", imuInitialized ? "YES ✓" : "NO (Odometry only)");
        telemetry.addData("", "");
        telemetry.addData("Current Position", "");
        telemetry.addData("  X", "%.1f mm", pos.getX(DistanceUnit.MM));
        telemetry.addData("  Y", "%.1f mm", pos.getY(DistanceUnit.MM));
        telemetry.addData("  H", "%.1f°", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("", "");
        telemetry.addData("⚠ IMPORTANT", "");
        if (!imuInitialized) {
            telemetry.addData("", "IMU NOT ACTIVE!");
            telemetry.addData("", "Heading from odometry only");
            telemetry.addData("", "Expect rotational drift");
        } else {
            telemetry.addData("", "IMU active - good heading accuracy");
        }
        telemetry.addData("", "");
        telemetry.addData("Press START to begin", "");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Track loop performance
        double currentLoopTime = runtime.milliseconds() - lastLoopTime;
        lastLoopTime = runtime.milliseconds();
        loopCount++;
        avgLoopTime = ((avgLoopTime * (loopCount - 1)) + currentLoopTime) / loopCount;

        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();
        double curX = pos.getX(DistanceUnit.MM);
        double curY = pos.getY(DistanceUnit.MM);
        double curH = pos.getHeading(AngleUnit.DEGREES);

        // Handle button presses (with debouncing)
        if (gamepad1.dpad_up && !lastDpadUp && !isTesting) {
            startTest("Forward 500mm", 0, TEST_SPEED, 0, curX, curY, curH);
        }
        if (gamepad1.dpad_down && !lastDpadDown && !isTesting) {
            startTest("Backward 500mm", 0, -TEST_SPEED, 0, curX, curY, curH);
        }
        if (gamepad1.dpad_left && !lastDpadLeft && !isTesting) {
            startTest("Strafe Left 500mm", -TEST_SPEED, 0, 0, curX, curY, curH);
        }
        if (gamepad1.dpad_right && !lastDpadRight && !isTesting) {
            startTest("Strafe Right 500mm", TEST_SPEED, 0, 0, curX, curY, curH);
        }
        if (gamepad1.x && !lastX && !isTesting) {
            startTest("Rotate CW 90°", 0, 0, TEST_SPEED, curX, curY, curH);
        }
        if (gamepad1.b && !lastB && !isTesting) {
            startTest("Rotate CCW 90°", 0, 0, -TEST_SPEED, curX, curY, curH);
        }
        if (gamepad1.left_bumper && !lastLeftBumper && !isTesting) {
            startComplexTest(ComplexTest.ROTATION_360, curX, curY, curH);
        }
        if (gamepad1.right_bumper && !lastRightBumper && !isTesting) {
            startComplexTest(ComplexTest.FIGURE_8, curX, curY, curH);
        }
        if (gamepad1.a && !lastA) {
            resetPosition();
        }
        if (gamepad1.y && !lastY) {
            cycleDisplayMode();
        }
        if (gamepad1.start && !lastStart) {
            saveReference(curX, curY, curH);
        }
        if (gamepad1.back && !lastBack && hasReference) {
            showReferenceDrift(curX, curY, curH);
        }

        // Update button states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastA = gamepad1.a;
        lastY = gamepad1.y;
        lastStart = gamepad1.start;
        lastBack = gamepad1.back;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;

        // Handle complex tests
        if (complexTest != ComplexTest.NONE) {
            handleComplexTest(curX, curY, curH);
        }

        // Run active test
        if (isTesting && complexTest == ComplexTest.NONE && testTimer.seconds() >= 3.0) {
            stopMotors();
            isTesting = false;

            // Calculate displacement
            double deltaX = curX - testStartX;
            double deltaY = curY - testStartY;
            double deltaH = curH - testStartH;

            // Normalize heading difference
            while (deltaH > 180) deltaH -= 360;
            while (deltaH <= -180) deltaH += 360;

            telemetry.addData("═══ TEST COMPLETE ═══", "");
            telemetry.addData("Test", currentTest);
            telemetry.addData("", "");
            telemetry.addData("Expected Movement", "");
            if (currentTest.contains("Rotate")) {
                telemetry.addData("  Rotation", "%.0f°", TEST_ANGLE);
            } else {
                telemetry.addData("  Distance", "%.0f mm", TEST_DISTANCE);
            }
            telemetry.addData("", "");
            telemetry.addData("Measured by Odometry", "");
            telemetry.addData("  ΔX", "%.1f mm", deltaX);
            telemetry.addData("  ΔY", "%.1f mm", deltaY);
            telemetry.addData("  ΔH", "%.1f°", deltaH);
            telemetry.addData("", "");

            // Diagnose issues
            if (currentTest.contains("Forward") && Math.abs(deltaY) > 50) {
                telemetry.addData("⚠ Issue", "Forward movement has Y drift");
                telemetry.addData("  Likely cause", "Encoder directions wrong");
            }
            if (currentTest.contains("Rotate") && (Math.abs(deltaX) > 50 || Math.abs(deltaY) > 50)) {
                telemetry.addData("⚠ Issue", "Rotation causes X/Y drift");
                telemetry.addData("  Likely cause", "Pod offsets incorrect");
            }
            if (currentTest.contains("Strafe") && Math.abs(deltaX) > 50) {
                telemetry.addData("⚠ Issue", "Strafe has X drift");
                telemetry.addData("  Likely cause", "Check encoder directions");
            }

            telemetry.addData("", "");
            telemetry.addData("⚠ Action Required", "Measure actual movement");
            telemetry.addData("", "Compare to odometry reading");
        }

        // Display based on mode
        if (!isTesting && complexTest == ComplexTest.NONE) {
            displayCurrentStatus(curX, curY, curH);
        } else if (complexTest == ComplexTest.NONE) {
            displayTestProgress(curX, curY, curH);
        }

        telemetry.update();
    }

    private void displayCurrentStatus(double curX, double curY, double curH) {
        telemetry.addData("═══ MODE: " + displayMode + " ═══", "");
        telemetry.addData("", "");

        switch (displayMode) {
            case BASIC:
                telemetry.addData("Current Position", "");
                telemetry.addData("  X", "%.1f mm", curX);
                telemetry.addData("  Y", "%.1f mm", curY);
                telemetry.addData("  H", "%.1f°", curH);
                telemetry.addData("", "");
                telemetry.addData("Quick Tests", "");
                telemetry.addData("  D-Pad", "Drive tests (500mm)");
                telemetry.addData("  X/B", "Rotation tests (90°)");
                telemetry.addData("", "");
                telemetry.addData("Advanced Tests", "");
                telemetry.addData("  L-Bumper", "360° rotation test");
                telemetry.addData("  R-Bumper", "Figure-8 pattern");
                telemetry.addData("", "");
                telemetry.addData("Utilities", "");
                telemetry.addData("  A", "Reset position");
                telemetry.addData("  Y", "Change display mode");
                telemetry.addData("  START", "Save reference");
                if (hasReference) {
                    telemetry.addData("  BACK", "Check drift from ref");
                }
                break;

            case DETAILED:
                Pose2D vel = pinpoint.getVelocity();
                telemetry.addData("Position", "");
                telemetry.addData("  X", "%.1f mm", curX);
                telemetry.addData("  Y", "%.1f mm", curY);
                telemetry.addData("  H", "%.1f°", curH);
                telemetry.addData("", "");
                telemetry.addData("Velocity", "");
                telemetry.addData("  X", "%.1f mm/s", vel.getX(DistanceUnit.MM));
                telemetry.addData("  Y", "%.1f mm/s", vel.getY(DistanceUnit.MM));
                telemetry.addData("  H", "%.1f°/s", vel.getHeading(AngleUnit.DEGREES));
                telemetry.addData("", "");
                telemetry.addData("Performance", "");
                telemetry.addData("  Loop Time", "%.1f ms", avgLoopTime);
                telemetry.addData("  Loop Hz", "%.1f", 1000.0 / avgLoopTime);
                telemetry.addData("  Loops", loopCount);
                break;

            case ENCODER_RAW:
                telemetry.addData("Position", "");
                telemetry.addData("  X", "%.1f mm", curX);
                telemetry.addData("  Y", "%.1f mm", curY);
                telemetry.addData("  H", "%.1f°", curH);
                telemetry.addData("", "");
                telemetry.addData("Raw Encoder Values", "");
                telemetry.addData("  X Pod", pinpoint.getEncoderX());
                telemetry.addData("  Y Pod", pinpoint.getEncoderY());
                telemetry.addData("", "");
                telemetry.addData("Pod Configuration", "");
                telemetry.addData("  X Offset", "%.1f mm", X_POD_OFFSET);
                telemetry.addData("  Y Offset", "%.1f mm", Y_POD_OFFSET);
                telemetry.addData("", "");
                telemetry.addData("Watch These Values", "");
                telemetry.addData("", "Move robot manually");
                telemetry.addData("", "Verify encoder changes match");
                break;

            case IMU_STATUS:
                telemetry.addData("IMU Status", imuStatus);
                telemetry.addData("IMU Active", imuInitialized ? "YES ✓" : "NO ✗");
                telemetry.addData("", "");
                telemetry.addData("Heading Source", "");
                if (imuInitialized) {
                    telemetry.addData("  Primary", "IMU Gyroscope ✓");
                    telemetry.addData("  Backup", "Odometry pods");
                    telemetry.addData("", "");
                    telemetry.addData("Expected Accuracy", "Excellent");
                    telemetry.addData("  Rotation drift", "Minimal");
                    telemetry.addData("  Long autonomy", "Reliable");
                } else {
                    telemetry.addData("  Primary", "Odometry pods only ✗");
                    telemetry.addData("  Backup", "None");
                    telemetry.addData("", "");
                    telemetry.addData("⚠ Warning", "No IMU detected");
                    telemetry.addData("  Rotation drift", "HIGH RISK");
                    telemetry.addData("  Long autonomy", "Will drift");
                    telemetry.addData("", "");
                    telemetry.addData("Fix", "Add pinpoint.resetPosAndIMU()");
                    telemetry.addData("", "to your init() method");
                }
                telemetry.addData("", "");
                telemetry.addData("Current Heading", "%.2f°", curH);
                telemetry.addData("", "");
                telemetry.addData("Test Recommendation", "");
                telemetry.addData("", "Run 360° rotation test");
                telemetry.addData("", "(Left Bumper)");
                if (!imuInitialized) {
                    telemetry.addData("", "Expected: Significant drift");
                } else {
                    telemetry.addData("", "Expected: Return to 0°");
                }
                break;
        }

        telemetry.addData("", "");
        telemetry.addData("Press Y", "Cycle display modes");
    }

    private void displayTestProgress(double curX, double curY, double curH) {
        telemetry.addData("═══ RUNNING TEST ═══", "");
        telemetry.addData("Test", currentTest);
        telemetry.addData("Time", "%.1f / 3.0 sec", testTimer.seconds());
        telemetry.addData("", "");
        telemetry.addData("Start Position", "");
        telemetry.addData("  X", "%.1f mm", testStartX);
        telemetry.addData("  Y", "%.1f mm", testStartY);
        telemetry.addData("  H", "%.1f°", testStartH);
        telemetry.addData("", "");
        telemetry.addData("Current Position", "");
        telemetry.addData("  X", "%.1f mm", curX);
        telemetry.addData("  Y", "%.1f mm", curY);
        telemetry.addData("  H", "%.1f°", curH);
        telemetry.addData("", "");
        telemetry.addData("Movement So Far", "");
        telemetry.addData("  ΔX", "%.1f mm", curX - testStartX);
        telemetry.addData("  ΔY", "%.1f mm", curY - testStartY);
        telemetry.addData("  ΔH", "%.1f°", curH - testStartH);
    }

    private void startTest(String testName, double strafe, double forward, double turn,
                           double startX, double startY, double startH) {
        currentTest = testName;
        testStartX = startX;
        testStartY = startY;
        testStartH = startH;

        // Calculate mecanum wheel powers
        double lf = forward - strafe - turn;
        double rf = forward + strafe + turn;
        double lr = forward + strafe - turn;
        double rr = forward - strafe + turn;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))));
        lf /= max;
        rf /= max;
        lr /= max;
        rr /= max;

        FrontLeft.setPower(lf);
        RearLeft.setPower(lr);
        FrontRight.setPower(rf);
        RearRight.setPower(rr);

        testTimer.reset();
        isTesting = true;
    }

    private void startComplexTest(ComplexTest test, double startX, double startY, double startH) {
        complexTest = test;
        testPhase = 0;
        testStartX = startX;
        testStartY = startY;
        testStartH = startH;
        testTimer.reset();

        if (test == ComplexTest.ROTATION_360) {
            currentTest = "360° Rotation Test";
        } else if (test == ComplexTest.FIGURE_8) {
            currentTest = "Figure-8 Pattern Test";
        }
    }

    private void handleComplexTest(double curX, double curY, double curH) {
        if (complexTest == ComplexTest.ROTATION_360) {
            // Rotate for 6 seconds (should be ~360°)
            if (testTimer.seconds() < 6.0) {
                double lf = -ROTATION_TEST_SPEED;
                double rf = ROTATION_TEST_SPEED;
                double lr = -ROTATION_TEST_SPEED;
                double rr = ROTATION_TEST_SPEED;

                FrontLeft.setPower(lf);
                RearLeft.setPower(lr);
                FrontRight.setPower(rf);
                RearRight.setPower(rr);

                telemetry.addData("═══ 360° ROTATION TEST ═══", "");
                telemetry.addData("Time", "%.1f / 6.0 sec", testTimer.seconds());
                telemetry.addData("", "");
                telemetry.addData("Start Heading", "%.1f°", testStartH);
                telemetry.addData("Current Heading", "%.1f°", curH);
                telemetry.addData("Rotation", "%.1f°", curH - testStartH);
                telemetry.addData("", "");
                telemetry.addData("Position Drift", "");
                telemetry.addData("  ΔX", "%.1f mm", curX - testStartX);
                telemetry.addData("  ΔY", "%.1f mm", curY - testStartY);
                telemetry.addData("", "");
                if (!imuInitialized) {
                    telemetry.addData("⚠ Note", "IMU not active");
                    telemetry.addData("", "Expect heading drift");
                }
            } else {
                stopMotors();
                double headingChange = curH - testStartH;
                double xDrift = curX - testStartX;
                double yDrift = curY - testStartY;

                telemetry.addData("═══ TEST COMPLETE ═══", "");
                telemetry.addData("", "");
                telemetry.addData("Expected", "360° rotation");
                telemetry.addData("Measured", "%.1f°", headingChange);
                telemetry.addData("Error", "%.1f°", Math.abs(headingChange - 360));
                telemetry.addData("", "");
                telemetry.addData("Position Drift", "");
                telemetry.addData("  X", "%.1f mm", xDrift);
                telemetry.addData("  Y", "%.1f mm", yDrift);
                telemetry.addData("  Total", "%.1f mm", Math.hypot(xDrift, yDrift));
                telemetry.addData("", "");

                if (Math.abs(headingChange - 360) > 10) {
                    telemetry.addData("⚠ Issue", "Large heading error");
                    if (!imuInitialized) {
                        telemetry.addData("  Likely cause", "No IMU - enable it!");
                    } else {
                        telemetry.addData("  Likely cause", "IMU calibration issue");
                    }
                }

                if (Math.hypot(xDrift, yDrift) > 100) {
                    telemetry.addData("⚠ Issue", "Position drifted during rotation");
                    telemetry.addData("  Likely cause", "Pod offsets wrong");
                }

                telemetry.addData("", "");
                telemetry.addData("Press A", "To continue testing");

                if (gamepad1.a) {
                    complexTest = ComplexTest.NONE;
                }
            }
        }
    }

    private void stopMotors() {
        FrontLeft.setPower(0);
        RearLeft.setPower(0);
        FrontRight.setPower(0);
        RearRight.setPower(0);
    }

    private void resetPosition() {
        pinpoint.setPosX(0, DistanceUnit.MM);
        pinpoint.setPosY(0, DistanceUnit.MM);
        pinpoint.setHeading(0, AngleUnit.DEGREES);
        pinpoint.update();

        currentTest = "Position Reset to Origin";
        loopCount = 0;
        avgLoopTime = 0;
    }

    private void cycleDisplayMode() {
        switch (displayMode) {
            case BASIC:
                displayMode = DisplayMode.DETAILED;
                break;
            case DETAILED:
                displayMode = DisplayMode.ENCODER_RAW;
                break;
            case ENCODER_RAW:
                displayMode = DisplayMode.IMU_STATUS;
                break;
            case IMU_STATUS:
                displayMode = DisplayMode.BASIC;
                break;
        }
    }

    private void saveReference(double curX, double curY, double curH) {
        refX = curX;
        refY = curY;
        refH = curH;
        hasReference = true;
    }

    private void showReferenceDrift(double curX, double curY, double curH) {
        double driftX = curX - refX;
        double driftY = curY - refY;
        double driftH = curH - refH;

        while (driftH > 180) driftH -= 360;
        while (driftH <= -180) driftH += 360;

        telemetry.addData("═══ DRIFT FROM REFERENCE ═══", "");
        telemetry.addData("", "");
        telemetry.addData("Reference Position", "");
        telemetry.addData("  X", "%.1f mm", refX);
        telemetry.addData("  Y", "%.1f mm", refY);
        telemetry.addData("  H", "%.1f°", refH);
        telemetry.addData("", "");
        telemetry.addData("Drift", "");
        telemetry.addData("  X", "%.1f mm", driftX);
        telemetry.addData("  Y", "%.1f mm", driftY);
        telemetry.addData("  H", "%.1f°", driftH);
        telemetry.addData("  Total XY", "%.1f mm", Math.hypot(driftX, driftY));
    }

    @Override
    public void stop() {
        stopMotors();
    }
}