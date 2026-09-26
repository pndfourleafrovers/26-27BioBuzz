package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Hard-coded "Near" autonomous for the Rumble - the short-range counterpart to AutoTurretFar.
 *
 * The robot starts against the wall right beside the goal: BLUE to the LEFT of the goal, RED to the
 * RIGHT of it (so from the robot's point of view the goal is to its right on blue, to its left on red -
 * the same handedness the Far auto uses).
 *
 * Sequence:
 *   1. Back up ~12 inches off the wall to the shooting spot.
 *   2. Camera-aim the turret at the goal (Limelight) and shoot the preloaded artifacts at NEAR launch
 *      velocity (CLOSE_LAUNCH_POWER, not the far-shot power).
 *   3. Back up another ~24 inches to clear the launch zone.
 *
 * Driving is encoder-based (DrivetrainSubsystem.runDriveToPosition) using ONLY the drive-motor encoders -
 * the Pinpoint odometry pods are never touched. There is no collecting in this auto, so the intake only
 * runs during the feed itself.
 *
 * Shooting matches this bot's mechanism (no kicker servo): with the flywheel spun up, the intake + hatch
 * run together to push artifacts into the flywheel. Whenever the intake is running but we are NOT
 * shooting, the hatch runs at SLOW_HATCH_VELOCITY (opposite direction) so artifacts are held back rather
 * than fed into a flywheel that isn't ready.
 *
 * This base is abstract (not registered) so only the concrete alliance variants
 * (AutoTurretNearBlue / AutoTurretNearRed) appear on the Driver Station.
 */
public abstract class AutoTurretNear extends LinearOpMode {

    /** @return the AprilTag ID of THIS alliance's goal (20 = blue, 24 = red). */
    protected abstract int getTargetTagId();

    /**
     * @return +1 if the goal is to the RIGHT of the turret's centered/straight-ahead direction from the
     *         starting pose, -1 if to the LEFT. Used to seed the turret near the goal before the first
     *         aim instead of starting cold from dead-center. If the turret visibly swings AWAY from the
     *         goal during init, this sign is backwards for that alliance - flip it.
     */
    protected abstract double aimSeedSign();

    // ---------------------------------------------------------------- Tunables (edit on the field)
    // Launch - NEAR shot. These are the close-range pair, matching what the old Near autos used.
    private final double LAUNCH_POWER       = CLOSE_LAUNCH_POWER;   // 1370
    private final double LAUNCH_MIN_VEL     = CLOSE_MIN_VELOCITY;   // 1350

    // Drive geometry - INCHES. Both moves are backwards (away from the wall/goal).
    private final double DRIVE_SPEED        = 0.6;
    private final double BACK_TO_SHOOT_IN   = 40;    // off the wall to the shooting spot, needs fixing
    private final double BACK_TO_CLEAR_IN   = 18;    // further back, out of the launch zone, needs fixing

    // Aim + shoot.
    private final double AIM_TIMEOUT_S      = 6.0;   // fire anyway if never LOCKED, so a lost tag can't hang auto
    private final double FEED_TIME_S        = 1.75;  // intake+hatch ON long enough to push the preloads through
    private final double SEARCH_SPEED_MULT  = 8.0;   // multiplies turretSearchStep while sweeping for the tag
    // Consecutive frames with no tag before the turret gives up and sweeps. Sweeping on a single dropped
    // frame would fling the turret off a target it could still see, so ride out short dropouts.
    private final int MISSES_BEFORE_SEARCH  = 5;     // ~1s at AIM_STEP_SETTLE_MS
    // Pace of the aim loop. The camera rides ON the turret, so a correction only means anything once the
    // servo has physically moved AND the Limelight has produced a frame from the NEW angle. Running the
    // loop faster than that stacks corrections for an error it hasn't re-measured and the turret
    // oscillates instead of settling.
    private final long AIM_STEP_SETTLE_MS   = 200;
    // Chassis heading offset from the goal at the starting pose, in degrees. The robot sits right beside
    // the goal here, so this is likely LARGER than the Far auto's value - verify it with the TAG readout
    // on the Driver Station before the match and adjust.
    private final double INITIAL_AIM_OFFSET_DEG = 90;

    // Limelight pipeline index for the AprilTag goal pipeline (set in the LL web UI).
    private final int LIMELIGHT_PIPELINE    = 0;

    // Split long encoder moves into segments no longer than this, to stay inside the drivetrain's
    // hard 3-second per-move timeout (MOVEMENT_TIMEOUT_SECONDS).
    private final double MAX_SEGMENT_IN     = 24.0;
    // Pause between chained segments so the prior segment's STOP_AND_RESET_ENCODER actually latches
    // before the next segment sets a new RUN_TO_POSITION target.
    private final long SEGMENT_SETTLE_MS    = 150;

    // ---------------------------------------------------------------- Subsystems
    private DrivetrainSubsystem drivetrain;
    private LauncherSubsystemRookieBot launcher;
    private TurretSubsystem turret;
    private Limelight3A limelight;

    // Sweep direction, persisted so a search resumes smoothly instead of restarting each time.
    private double searchDir = 1.0;

    // Consecutive aim iterations with no tag in frame; reset the moment it is seen again.
    private int consecutiveMisses = 0;

    @Override
    public void runOpMode() {
        // --- Init ---
        drivetrain = new DrivetrainSubsystem(hardwareMap);          // ctor resets drive encoders
        launcher = new LauncherSubsystemRookieBot(hardwareMap);     // grabs Intake/Hatch/Launcher/LaunchAsst
        turret = new TurretSubsystem(hardwareMap);                  // ctor centers the turret at 150 deg

        // One-time coarse seed toward the goal, so the fine aimAtAngle loop starts from a small residual
        // error instead of the full offset. nudge() (not setTurretPosition()) because it also updates the
        // internal accumulator aimAtAngle() reads from.
        double seed = initialTurretSeed();
        turret.nudge(seed - TurretMid);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        drivetrain.stopAndResetEncoders();

        // Pre-start: report live whether the goal tag is ALREADY in frame from the seeded turret position,
        // so placement (or INITIAL_AIM_OFFSET_DEG / aimSeedSign) can be fixed before the match rather than
        // discovered after play.
        while (!isStarted() && !isStopRequested()) {
            Double initTx = extractTx(limelight.getLatestResult());
            telemetry.addData("Status", "Turret NEAR - tag %d", getTargetTagId());
            telemetry.addData("Turret seed", "pos=%.2f (~%.0f deg)", seed, seed * TurretSubsystem.SERVO_RANGE_DEGREES);
            if (initTx != null) {
                telemetry.addData("TAG", ">>> VISIBLE  tx=%+.1f deg <<<  only needs fine-tuning on start", initTx);
            } else {
                telemetry.addData("TAG", "NOT VISIBLE - re-aim the robot, or adjust INITIAL_AIM_OFFSET_DEG (now %.0f) / aimSeedSign()",
                        INITIAL_AIM_OFFSET_DEG);
            }
            telemetry.update();
            sleep(100);
        }

        if (!opModeIsActive()) { limelight.stop(); return; }

        // Spin the flywheel up at NEAR power now, so it is at speed by the time we finish backing up.
        launcher.setLauncherVelocity(LAUNCH_POWER);

        // 1. Back off the wall to the shooting spot.
        driveInches(DRIVE_SPEED, -BACK_TO_SHOOT_IN);

        // 2. Aim and fire the preloads.
        aimAndShoot();

        // 3. Back out of the launch zone.
        driveInches(DRIVE_SPEED, -BACK_TO_CLEAR_IN);

        // --- Cleanup ---
        launcher.stopAll();
        turret.centerTurret();
        limelight.stop();
    }

    /**
     * @return the turret position (0.0-1.0) to seed toward before aiming, from the estimated starting-pose
     *         offset (INITIAL_AIM_OFFSET_DEG) and which side the goal is on (aimSeedSign()).
     */
    private double initialTurretSeed() {
        double offsetPos = INITIAL_AIM_OFFSET_DEG / TurretSubsystem.SERVO_RANGE_DEGREES;
        // aimAtAngle decreases turretPos to swing toward a target with positive Limelight tx (tx positive
        // = target to the right), so "goal is right of center" seeds turretPos BELOW TurretMid.
        return TurretMid - (offsetPos * aimSeedSign());
    }

    // ---------------------------------------------------------------- Aim + shoot

    /**
     * Aims the turret at the goal and fires the preloads.
     *   1. Settle: wait until the turret is LOCKED and the flywheel is at NEAR speed, or AIM_TIMEOUT_S.
     *   2. Feed:   run intake + hatch together for FEED_TIME_S to push the artifacts into the flywheel,
     *              reverse-holding the hatch whenever the flywheel drops below speed.
     * Both phases keep closing the aim loop so the turret stays locked while firing.
     */
    private void aimAndShoot() {
        // Phase 1: settle.
        ElapsedTime settle = new ElapsedTime();
        while (opModeIsActive()) {
            boolean locked = aimOnce();
            boolean atSpeed = launcher.getLauncherVelocity() >= LAUNCH_MIN_VEL;
            logTelemetry("Aim", "settling  locked=%b atSpeed=%b t=%.2f", locked, atSpeed, settle.seconds());
            if ((locked && atSpeed) || settle.seconds() > AIM_TIMEOUT_S) break;
            sleep(AIM_STEP_SETTLE_MS);   // let the servo finish moving before re-measuring
        }

        // Phase 2: feed.
        launcher.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
        ElapsedTime feed = new ElapsedTime();
        while (opModeIsActive() && feed.seconds() < FEED_TIME_S) {
            aimOnce();                                              // keep tracking while firing
            if (launcher.getLauncherVelocity() >= LAUNCH_MIN_VEL) {
                launcher.setHatchVelocity(HATCH_VELOCITY);          // at speed: feed the flywheel
            } else {
                // Intake is still running here, so reverse-hold rather than stop - at 0 the intake would
                // keep pushing artifacts into a flywheel that isn't ready for them.
                launcher.setHatchVelocity(SLOW_HATCH_VELOCITY);
            }
            logTelemetry("Aim", "FIRING  t=%.2f  vel=%.0f", feed.seconds(), launcher.getLauncherVelocity());
            sleep(AIM_STEP_SETTLE_MS);
        }

        launcher.setHatchVelocity(0);
        launcher.setIntakeVelocity(0);
    }

    /**
     * One aim iteration. Closes the loop on the tag's horizontal angle when it is visible; holds position
     * through brief dropouts; only sweeps to search after MISSES_BEFORE_SEARCH consecutive misses.
     *
     * @return true if the turret is LOCKED on the tag this iteration.
     */
    private boolean aimOnce() {
        LLResult result = limelight.getLatestResult();
        logLimelightRaw(result);

        Double tx = extractTx(result);
        if (tx != null) {
            consecutiveMisses = 0;
            TurretSubsystem.AimStatus status = turret.aimAtAngle(tx);
            // tx is the live angle error; it should shrink toward zero across iterations. If it grows or
            // flips sign every step, the aim is diverging rather than settling slowly.
            logTelemetry("Track", "tx=%+.1f deg  status=%s  pos=%.3f", tx, status, turret.getCommandedPosition());
            return status == TurretSubsystem.AimStatus.LOCKED;
        }

        consecutiveMisses++;
        if (consecutiveMisses < MISSES_BEFORE_SEARCH) {
            logTelemetry("Track", "no tag (%d/%d) - holding aim", consecutiveMisses, MISSES_BEFORE_SEARCH);
            return false;
        }

        // Genuinely lost: sweep, reversing direction at the travel limits.
        double pos = turret.getCommandedPosition();
        if (pos <= TurretMin + 0.01) {
            searchDir = 1.0;
        } else if (pos >= TurretMax - 0.01) {
            searchDir = -1.0;
        }
        logTelemetry("Track", "no tag (%d) - SEARCHING  pos=%.3f", consecutiveMisses, pos);
        turret.nudge(turretSearchStep * SEARCH_SPEED_MULT * searchDir);
        return false;
    }

    /**
     * @return the horizontal angle (degrees) to THIS alliance's goal tag in the given Limelight result,
     *         or null if that tag is not present.
     */
    private Double extractTx(LLResult result) {
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == getTargetTagId()) {
                    return fr.getTargetXDegrees();
                }
            }
        }
        return null;
    }

    /** Reports what the Limelight is actually producing, to separate camera/pipeline problems from aim problems. */
    private void logLimelightRaw(LLResult result) {
        boolean valid = result != null && result.isValid();
        StringBuilder ids = new StringBuilder();
        if (valid) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                ids.append(fr.getFiducialId()).append(" ");
            }
        }
        logTelemetry("Limelight", "valid=%b seenIDs=[%s] wantID=%d", valid, ids.toString().trim(), getTargetTagId());
    }

    /** addData under the given caption, then update(). Distinct captions so all lines stay visible. */
    private void logTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        telemetry.update();
    }

    // ---------------------------------------------------------------- Drive helpers

    /**
     * Blocking straight drive of the given signed distance (negative = backward). Split into segments no
     * longer than MAX_SEGMENT_IN so no single move exceeds the drivetrain's 3-second timeout, with a
     * pause between segments so each one's encoder reset has taken effect before the next target is set.
     */
    private void driveInches(double speed, double inches) {
        double remaining = inches;
        while (opModeIsActive() && Math.abs(remaining) > 0.1) {
            double step = (Math.abs(remaining) > MAX_SEGMENT_IN)
                    ? Math.copySign(MAX_SEGMENT_IN, remaining)
                    : remaining;
            while (opModeIsActive() && !drivetrain.runDriveToPosition(speed, step, 0, DistanceUnit.INCH)) {
                idle();
            }
            remaining -= step;
            if (opModeIsActive() && Math.abs(remaining) > 0.1) sleep(SEGMENT_SETTLE_MS);
        }
    }
}
