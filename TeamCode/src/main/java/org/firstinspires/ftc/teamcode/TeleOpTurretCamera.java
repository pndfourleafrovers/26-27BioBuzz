package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Full teleop with a camera-tracking turret (alliance-agnostic base class).
 *
 * The turret rides on two ganged servos owned by TurretSubsystem. A turret-mounted Limelight 3A
 * detects AprilTags; this op mode filters the detections for THIS alliance's goal tag (see
 * {@link #getTargetTagId()}) and uses that tag's horizontal angle to aim. When tracking is toggled ON
 * (left bumper), the turret closes the loop on that angle each frame until the target is centered,
 * then holds. When tracking is OFF or the target tag isn't visible, the turret holds its last position.
 * The driver keeps full drive control at all times.
 *
 * Concrete alliance variants (TeleOpTurretCameraBlue / TeleOpTurretCameraRed) supply the tag ID and
 * the @TeleOp registration. This base is abstract so it is not registered on the Driver Station.
 *
 * Drive/launcher/intake controls mirror TeleOpTurret.
 */

public abstract class TeleOpTurretCamera extends LinearOpMode {

    /** @return the AprilTag ID this op mode should lock onto (e.g. 20 = blue goal, 24 = red goal). */
    protected abstract int getTargetTagId();

    // Subsystems
    private DrivetrainSubsystem drivetrain;
     private LauncherSubsystemRookieBot launcherSubsystemRookieBot;
    private TurretSubsystem turretSubsystem;
    private Limelight3A limelight;

    // Drive state
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private static final double SLOW_MULTIPLIER = 0.35;

    // Turret tracking state
    private boolean trackingEnabled = false;

    // Launcher state
    private double minVelocity = TELE_CLOSE_MIN_LAUNCH;
    private boolean manualPowerOverride = true;

    @Override
    public void runOpMode() {

        drivetrain = new DrivetrainSubsystem(hardwareMap);
        launcherSubsystemRookieBot = new LauncherSubsystemRookieBot(hardwareMap);

        // TurretSubsystem owns ServoFront/ServoBack and centers the turret (150 deg) in its constructor.
        turretSubsystem = new TurretSubsystem(hardwareMap);

        // Limelight: start polling. getLatestResult() never returns null once started - check isValid().
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // TODO: set to the pipeline index for your target in the LL web UI
        limelight.start();

        drivetrain.stopAndResetEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        launcherSubsystemRookieBot.setHatchVelocity(0);

        while (opModeIsActive()) {

           //------------------------------------------------ GAMEPAD 1: DRIVE ------------------------
            double axial = gamepad1.left_stick_y;   // forward = negative
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            double forward = -axial;
            double strafe = -lateral;
            double turn = yaw;

            double driveMagnitude = Math.hypot(forward, strafe);
            if (driveMagnitude > 1.0) {
                forward /= driveMagnitude;
                strafe /= driveMagnitude;
            }

            double leftFrontPower = forward - strafe - turn;
            double rightFrontPower = forward + strafe + turn;
            double leftBackPower = forward + strafe - turn;
            double rightBackPower = forward - strafe + turn;

            double max = Math.max(1.0, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

            // Right bumper: hold for slow mode.
            slowMode = gamepad1.right_bumper;
            powerMultiplier = slowMode ? SLOW_MULTIPLIER : 1.0;

            drivetrain.setMotorPower(
                    leftFrontPower * powerMultiplier,
                    rightFrontPower * powerMultiplier,
                    leftBackPower * powerMultiplier,
                    rightBackPower * powerMultiplier);

            //------------------------------------------------ TURRET CAMERA TRACKING ------------------
            // Left bumper toggles tracking on/off.
            if (gamepad1.leftBumperWasPressed()) {
                trackingEnabled = !trackingEnabled;
            }

            // Find THIS alliance's goal tag among the Limelight's detections, and use its angle.
            Double targetTx = null; // null = our tag not currently seen
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == getTargetTagId()) {
                        targetTx = fr.getTargetXDegrees(); // horizontal angle to this tag (degrees)
                        break;
                    }
                }
            }

            if (trackingEnabled && targetTx != null) {
                TurretSubsystem.AimStatus status = turretSubsystem.aimAtAngle(targetTx);
                telemetry.addData("Turret", "%s  tag %d  tx=%.2f  pos=%.3f",
                        status, getTargetTagId(), targetTx, turretSubsystem.getCommandedPosition());
            } else if (trackingEnabled) {
                // Tag not seen: let the driver hunt for it with gamepad2's right stick (proportional).
                // The moment the tag reappears, the aim branch above takes over and locks.
                double searchInput = gamepad2.right_stick_x; // -1 (left) .. +1 (right)
                if (Math.abs(searchInput) > 0.05) {          // deadzone
                    // Flip the sign here if search moves opposite the stick (match aimAtAngle's sign).
                    turretSubsystem.nudge(searchInput * turretSearchStep);
                }
                telemetry.addData("Turret", "TRACKING ON - tag %d not seen, searching (stick=%.2f pos=%.3f)",
                        getTargetTagId(), searchInput, turretSubsystem.getCommandedPosition());
            } else {
                // Tracking OFF: lock the turret to center every loop, so it returns to (and holds) a
                // known position for consistency if something malfunctions mid-match.
                turretSubsystem.centerTurret();
                telemetry.addData("Turret", "tracking OFF - centered (pos=%.3f)",
                        turretSubsystem.getCommandedPosition());
            }

            //------------------------------------------------ GAMEPAD 1: INTAKE / HATCH ---------------
            //if x and right trigger is being pressed then fire?
            if (gamepad2.x) {
                launcherSubsystemRookieBot.setHatchVelocity(HATCH_VELOCITY);
                launcherSubsystemRookieBot.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
            } else if (gamepad1.right_trigger > 0.5) {
                launcherSubsystemRookieBot.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                launcherSubsystemRookieBot.setHatchVelocity(SLOW_HATCH_VELOCITY);
            } else {
                launcherSubsystemRookieBot.setIntakeVelocity(0);
                launcherSubsystemRookieBot.setHatchVelocity(0);
            }

            //------------------------------------------------ GAMEPAD 2: LAUNCHER ---------------------
            if (gamepad2.left_trigger > 0) {
                launcherSubsystemRookieBot.setLauncherVelocity(LaunchPower);
            } else {
                launcherSubsystemRookieBot.setLauncherVelocity(0);
            }

            if (gamepad2.dpad_up) {          // FAR
                LaunchPower = FAR_LAUNCH_POWER;
                minVelocity = FAR_MIN_VELOCITY;
            } else if (gamepad2.dpad_left) { // MIDDLE
                LaunchPower = MID_LAUNCH_POWER;
                minVelocity = MID_MIN_VELOCITY;
            } else if (gamepad2.dpad_down) { // CLOSE
                LaunchPower = TELE_CLOSE_LAUNCH;
                minVelocity = TELE_CLOSE_MIN_LAUNCH;
            }

            if (gamepad2.yWasPressed()) {
                manualPowerOverride = !manualPowerOverride;
            }

            //------------------------------------------------ TELEMETRY -------------------------------
            LLStatus llStatus = limelight.getStatus();
            telemetry.addData("Limelight", "pipeline %d, %.0f fps, temp %.0fC",
                    llStatus.getPipelineIndex(), llStatus.getFps(), llStatus.getTemp());
            telemetry.addData("Tracking", trackingEnabled ? "ON" : "OFF");

            double launcherVelocity = launcherSubsystemRookieBot.getLauncherVelocity();
            double launchAsstVelocity = launcherSubsystemRookieBot.getLaunchAsstVelocity();
            telemetry.addData("-----Launcher-----", "");
            telemetry.addData("Launch Power (target)", "%.0f", LaunchPower);
            telemetry.addData("Min Velocity", "%.0f", minVelocity);
            telemetry.addData("Launcher Velocity", "%.0f", launcherVelocity);
            telemetry.addData("Launch Asst Velocity", "%.0f", launchAsstVelocity);
            telemetry.addData("Launcher RPM", "%.0f", (launcherVelocity / TICKS_PER_REV) * 60);
            telemetry.addData("Intake Velocity", "%.0f", launcherSubsystemRookieBot.getIntakeVelocity());
            telemetry.update();
        }

        limelight.stop();
    }
}
