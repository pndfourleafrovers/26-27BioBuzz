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
 * Hard-coded "Far" autonomous for the Rumble, built for the case where odometry isn't trusted.
 *
 * The robot starts against the wall on the FAR side of the field from the goal, facing the goal.
 * Sequence:
 *   1. Camera-aim the turret at the goal (Limelight) and shoot the 3 preloaded balls.
 *   2. Then repeat until the auto period ends (or the OpMode is stopped): drive forward with the intake
 *      running, drive back the same distance to the shooting spot, re-aim, and shoot.
 * There is no fixed end state or park - the cycle just runs out the clock.
 *
 * Driving is encoder-based (DrivetrainSubsystem.runDriveToPosition / runStrafeToPosition) using ONLY
 * the four drive-motor encoders - the Pinpoint odometry pods are never touched.
 *
 * Shooting matches this bot's mechanism (no kicker servo): with the flywheel spun up, the intake + hatch
 * motors run together to push balls up into the flywheel (see TeleOpTurretCamera gamepad1.x).
 *
 * This base is abstract (not registered) so only the concrete alliance variants
 * (AutoTurretFarBlue / AutoTurretFarRed) appear on the Driver Station. Subclasses supply the goal
 * AprilTag ID and the strafe mirror sign.
 */
public abstract class AutoTurretFar extends LinearOpMode {

    /** @return the AprilTag ID of THIS alliance's goal (20 = blue, 24 = red). */
    protected abstract int getTargetTagId();

    /** @return +1 or -1, to mirror any strafe used to line up on a ball row (red field is mirrored). */
    protected abstract double lateralSign();

    /**
     * @return +1 if the goal is to the RIGHT of the turret's centered/straight-ahead direction from the
     *         starting pose, -1 if to the LEFT. Used to seed the turret near the goal before the first
     *         aim instead of starting cold from dead-center. If the turret visibly swings AWAY from the
     *         goal during init, this sign is backwards for that alliance - flip it.
     */
    protected abstract double aimSeedSign();

    // ---------------------------------------------------------------- Tunables (edit on the field)
    // Launch (far shot). FAR_LAUNCH_POWER / FAR_MIN_VELOCITY come from Constants.
    private final double LAUNCH_POWER       = FAR_LAUNCH_POWER;   // flywheel target velocity
    private final double LAUNCH_MIN_VEL     = FAR_MIN_VELOCITY;   // "at speed" threshold before feeding

    // Drive geometry - INCHES. Tune these against the actual field.
    private final double DRIVE_SPEED        = 0.6;   // travel moves
    private final double INTAKE_DRIVE_SPEED = 0.35;  // slower creep outward while collecting
    // Forward distance driven each collect cycle. The return trip uses this same value, so the robot
    // ends every cycle back at the shooting spot. This is the main knob to tune on the field.
    private final double CYCLE_FWD_IN       = 58;
    private final double LINE_STRAFE_IN     = 0;     // strafe to align on a row (multiplied by lateralSign())

    // Aim + shoot.
    private final double INITIAL_AIM_TIMEOUT_S = 6.0; // 1st shot: turret starts centered, needs to search+settle
    // Later shots: the turret is restored to the last locked position first, so it usually only needs a
    // small correction - but allow enough time to re-search if drive drift pushed the tag out of frame.
    private final double REAIM_TIMEOUT_S    = 2.5;
    private final double FEED_TIME_S        = 1.75;   // intake+hatch ON long enough to push all 3 balls through
    private final double SEARCH_SPEED_MULT  = 8.0;    // multiplies turretSearchStep while sweeping for the tag
    // Consecutive frames with no tag before the turret gives up and sweeps. The seed should leave the tag
    // already in view, so a miss is almost always a dropped/blurred frame rather than a genuinely lost
    // target. Sweeping on a single miss flings the turret ~12 deg off a target it could still see, which
    // then really does lose it - so hold position through short dropouts and only search as a last resort.
    private final int MISSES_BEFORE_SEARCH  = 5;      // ~1s at AIM_STEP_SETTLE_MS
    // Pace of the aim loop. The camera rides ON the turret, so each correction is only meaningful once
    // the servo has physically moved AND the Limelight has produced a frame from the NEW angle. Without
    // this pause the loop spins at 50-100+ Hz against a servo that needs hundreds of ms to swing, so it
    // stacks dozens of corrections for an error it hasn't re-measured, winds into a travel limit, then
    // slams back the other way - the "swings back and forth, then fires at a random angle" behavior.
    // Raise this if it still oscillates; lower it only if locking is too slow.
    private final long AIM_STEP_SETTLE_MS   = 200;
    // Chassis heading offset from the goal at the starting pose, in degrees. Re-measure this if the
    // starting spot changes.
    private final double INITIAL_AIM_OFFSET_DEG = 90;

    // Limelight pipeline index for the AprilTag goal pipeline (set in the LL web UI; see teleop TODO).
    private final int LIMELIGHT_PIPELINE    = 0;

    // Split long encoder moves into segments no longer than this, to stay inside the drivetrain's
    // hard 3-second per-move timeout (MOVEMENT_TIMEOUT_SECONDS).
    private final double MAX_SEGMENT_IN     = 24.0;
    // Pause between chained segments so the prior segment's STOP_AND_RESET_ENCODER actually latches on
    // the hardware before the next segment sets a new RUN_TO_POSITION target. Without this gap, back-to-
    // back calls can read a stale (not-yet-zeroed) encoder count and overshoot.
    private final long SEGMENT_SETTLE_MS    = 150;

    // ---------------------------------------------------------------- Subsystems
    private DrivetrainSubsystem drivetrain;
    private LauncherSubsystemRookieBot launcher;
    private TurretSubsystem turret;
    private Limelight3A limelight;

    // Persists the sweep direction across aimOnce() calls so a search resumes smoothly instead of
    // restarting from scratch each time the tag drops out of view.
    private double searchDir = 1.0;

    // Turret position at the moment of the most recent successful lock; -1 until the first lock.
    // Each aim starts by returning here. Without it, a cycle whose search failed leaves the turret
    // parked wherever the sweep stopped, so the NEXT cycle starts from a worse position than the last -
    // failures compound across cycles instead of each cycle getting a fresh, good starting guess.
    private double lastLockedPos = -1;

    // Consecutive aim iterations with no tag in frame; reset the moment it is seen again.
    private int consecutiveMisses = 0;

    @Override
    public void runOpMode() {
        // --- Init ---
        drivetrain = new DrivetrainSubsystem(hardwareMap);          // ctor resets drive encoders
        launcher = new LauncherSubsystemRookieBot(hardwareMap);     // grabs Intake/Hatch/Launcher/LaunchAsst
        turret = new TurretSubsystem(hardwareMap);                  // ctor centers the turret at 150 deg

        // One-time coarse seed toward the goal, so the fine aimAtAngle loop starts from a small residual
        // error (like a teleop driver roughly pointing the chassis first) instead of the full offset.
        // nudge() (not setTurretPosition()) because it also updates TurretSubsystem's internal position
        // accumulator that aimAtAngle() reads from - setTurretPosition() alone would get silently
        // overwritten by the first aimAtAngle() call, which starts from the stale centered accumulator.
        double seed = initialTurretSeed();
        turret.nudge(seed - TurretMid);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        drivetrain.stopAndResetEncoders();

        // Pre-start: report live whether the goal tag is ALREADY in frame from the seeded turret position.
        // The seed is meant to leave the tag visible so the first shot only needs fine-tuning, so the drive
        // team can confirm that here - and fix placement (or INITIAL_AIM_OFFSET_DEG / aimSeedSign) before
        // the match - instead of finding out after play that it has to search.
        while (!isStarted() && !isStopRequested()) {
            Double initTx = extractTx(limelight.getLatestResult());
            telemetry.addData("Status", "Turret Far - tag %d", getTargetTagId());
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

        // Spin the flywheel up once and leave it running for the whole auto (avoids re-spin lag).
        launcher.setLauncherVelocity(LAUNCH_POWER);

        // 1. Preloads.
        aimAndShoot(INITIAL_AIM_TIMEOUT_S);

        // 2. Collect-and-shoot cycles until the auto period ends or the OpMode is stopped. Every helper
        //    below also checks opModeIsActive(), so a stop partway through a cycle exits promptly rather
        //    than finishing the drive first.
        int cycle = 1;
        while (opModeIsActive()) {
            logTelemetry("Cycle", "#%d - collecting", cycle);
            collectRun(CYCLE_FWD_IN);
            if (!opModeIsActive()) break;
            logTelemetry("Cycle", "#%d - shooting", cycle);
            aimAndShoot(REAIM_TIMEOUT_S);
            cycle++;
        }

        // --- Cleanup ---
        launcher.stopAll();
        turret.centerTurret();
        limelight.stop();
    }

    /**
     * @return the turret position (0.0-1.0) to seed toward before the first aim, based on the estimated
     *         starting-pose offset (INITIAL_AIM_OFFSET_DEG) and which side the goal is on (aimSeedSign()).
     */
    private double initialTurretSeed() {
        double offsetPos = INITIAL_AIM_OFFSET_DEG / TurretSubsystem.SERVO_RANGE_DEGREES;
        // aimAtAngle decreases turretPos to swing the turret toward a target with positive Limelight tx
        // (tx positive = target to the right), so "goal is right of center" seeds turretPos BELOW TurretMid.
        return TurretMid - (offsetPos * aimSeedSign());
    }

    // ---------------------------------------------------------------- Aim + shoot

    /**
     * Aims the turret at the goal and fires whatever balls are loaded. Two phases, both of which keep
     * closing the aim loop every iteration so the turret stays locked (or keeps searching) while firing:
     *   1. Settle: wait until the turret is LOCKED and the flywheel is at speed, or timeoutS passes.
     *   2. Feed:   run intake + hatch together for FEED_TIME_S to push the balls up into the flywheel,
     *              momentarily pausing the hatch if the flywheel dips below speed ("velo lock").
     *
     * @param timeoutS how long to try to settle before firing anyway (a lost/never-seen tag can't hang
     *                 the match). Pass a longer value for the first call (turret starts centered with no
     *                 idea where the goal is) and a shorter value for later calls (already roughly aimed).
     */
    private void aimAndShoot(double timeoutS) {
        // Start from the last known-good aim rather than wherever a previous failed search left the
        // turret, so each cycle gets the best available starting guess instead of inheriting drift.
        if (lastLockedPos >= 0) {
            double delta = lastLockedPos - turret.getCommandedPosition();
            if (Math.abs(delta) > 0.001) {
                turret.nudge(delta);
                sleep(AIM_STEP_SETTLE_MS);   // let the servo arrive before the first measurement
            }
        }

        // Phase 1: settle.
        ElapsedTime settle = new ElapsedTime();
        while (opModeIsActive()) {
            boolean locked = aimOnce();
            boolean atSpeed = launcher.getLauncherVelocity() >= LAUNCH_MIN_VEL;
            logTelemetry("Aim", "settling  locked=%b atSpeed=%b t=%.2f", locked, atSpeed, settle.seconds());
            if ((locked && atSpeed) || settle.seconds() > timeoutS) break;
            sleep(AIM_STEP_SETTLE_MS);   // let the servo finish moving before re-measuring; see the constant
        }

        // Phase 2: feed.
        launcher.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
        ElapsedTime feed = new ElapsedTime();
        while (opModeIsActive() && feed.seconds() < FEED_TIME_S) {
            aimOnce();                                              // keep tracking while firing
            if (launcher.getLauncherVelocity() >= LAUNCH_MIN_VEL) {
                launcher.setHatchVelocity(HATCH_VELOCITY);          // at speed: feed the flywheel
            } else {
                // Flywheel dipped below speed. The intake is still running here, so the hatch has to
                // reverse-hold rather than just stop - at 0 the intake would keep pushing artifacts up
                // into a flywheel that isn't ready for them.
                launcher.setHatchVelocity(SLOW_HATCH_VELOCITY);
            }
            logTelemetry("Aim", "FIRING  t=%.2f  vel=%.0f", feed.seconds(), launcher.getLauncherVelocity());
            sleep(AIM_STEP_SETTLE_MS);   // same pacing as the settle phase, so mid-fire tracking can't wind up
        }

        launcher.setHatchVelocity(0);
        launcher.setIntakeVelocity(0);
    }

    /**
     * One aim iteration. If this alliance's tag is visible, closes the loop on its horizontal angle
     * (aimAtAngle) and reports whether the turret is within the deadband. If the tag is NOT visible, the
     * turret actively sweeps across its travel range to search for it (mirrors the driver-controlled
     * search in TeleOpTurretCamera, but automatic) - without this, a turret that starts centered with the
     * goal out of view would just sit still until timeout and never lock.
     *
     * Also logs raw Limelight status every call (see logLimelightRaw) so a field run shows exactly
     * whether the camera is producing valid results and what tag IDs (if any) it sees.
     *
     * @return true if the turret is LOCKED on the tag this iteration, false otherwise (incl. searching).
     */
    private boolean aimOnce() {
        LLResult result = limelight.getLatestResult();
        logLimelightRaw(result);

        Double tx = extractTx(result);
        if (tx != null) {
            consecutiveMisses = 0;
            TurretSubsystem.AimStatus status = turret.aimAtAngle(tx);
            boolean locked = (status == TurretSubsystem.AimStatus.LOCKED);
            if (locked) {
                lastLockedPos = turret.getCommandedPosition();   // remember a good aim for the next cycle
            }
            // tx is the live angle error in degrees; watching it shrink 45 -> 18 -> 7 -> 2 across
            // iterations confirms the loop is converging. If it grows or flips sign every step instead,
            // the aim is diverging (wrong sign) rather than just settling slowly.
            logTelemetry("Track", "tx=%+.1f deg  status=%s  pos=%.3f", tx, status, turret.getCommandedPosition());
            return locked;
        }

        // Tag not seen this frame. Hold position through brief dropouts (see MISSES_BEFORE_SEARCH) - the
        // turret stays where it was, so the next good frame resumes fine-tuning from the right place.
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
     * @return the horizontal angle (degrees) to THIS alliance's goal tag from the given Limelight result,
     *         or null if that tag is not present in it.
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

    /**
     * Prints exactly what the Limelight is producing this frame: whether the result is valid, and every
     * fiducial ID it currently sees. Use this on the Driver Station to distinguish "camera/pipeline not
     * producing results at all" from "producing results, but not the tag we want" from "sees our tag but
     * the turret still won't lock" - each points to a different fix (pipeline config, tag/alliance
     * mismatch, or an aim-loop bug), so don't guess without checking this first.
     */
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

    /**
     * telemetry.addData under the given caption, then update(). No throttling: the aim loop is now paced
     * at AIM_STEP_SETTLE_MS, so update()'s network cost is negligible and every step should be visible on
     * the Driver Station. Distinct captions ("Aim"/"Track"/"Limelight") so all stay visible instead of
     * overwriting each other - Telemetry keeps the latest value per caption and sends them all on update().
     */
    private void logTelemetry(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
        telemetry.update();
    }

    // ---------------------------------------------------------------- Drive helpers

    /**
     * One collect run: drive forward with the intake running, then drive back the SAME distance so the
     * robot ends where it started - the shooting spot.
     *
     * Intake and hatch are both left running afterward, and deliberately so: the hatch runs at
     * SLOW_HATCH_VELOCITY, which is the OPPOSITE direction from the HATCH_VELOCITY used to feed the
     * flywheel, so it pushes back against the collected artifacts and keeps them in the robot instead of
     * letting them creep into the launcher. This mirrors what teleop does while collecting (see the
     * right_trigger branch in TeleOpTurretCamera). Leaving both running means the artifacts stay retained
     * through the drive back AND through the aim/settle phase, until aimAndShoot's feed phase reverses
     * the hatch to actually shoot them.
     *
     * @param fwdIn forward inches to travel while collecting; the return trip uses the same value.
     */
    private void collectRun(double fwdIn) {
        launcher.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
        launcher.setHatchVelocity(SLOW_HATCH_VELOCITY);   // reverse-hold: keeps artifacts in the bot
        double strafe = LINE_STRAFE_IN * lateralSign();

        if (strafe != 0) { strafeInches(DRIVE_SPEED, strafe); settleGap(); }
        driveInches(INTAKE_DRIVE_SPEED, fwdIn);      // out, collecting
        settleGap();
        driveInches(DRIVE_SPEED, -fwdIn);            // back the same distance, to the shooting spot
        if (strafe != 0) { settleGap(); strafeInches(DRIVE_SPEED, -strafe); }
        // intake + reverse-hold hatch left ON on purpose (see javadoc) - they retain the artifacts until
        // aimAndShoot's feed phase reverses the hatch to fire.
    }

    /** Pause between chained drivetrain moves so the prior move's encoder reset has time to latch. */
    private void settleGap() {
        if (opModeIsActive()) sleep(SEGMENT_SETTLE_MS);
    }

    /**
     * Blocking straight drive of the given signed distance (negative = backward). Split into segments no
     * longer than MAX_SEGMENT_IN so no single encoder move exceeds the drivetrain's 3-second timeout.
     * A short pause follows each segment (see SEGMENT_SETTLE_MS) so the encoder reset from the segment
     * that just finished has actually taken effect before the next segment's target is set.
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

    /** Blocking strafe of the given signed distance (positive = right). Same settle-gap reasoning as driveInches. */
    private void strafeInches(double speed, double inches) {
        double remaining = inches;
        while (opModeIsActive() && Math.abs(remaining) > 0.1) {
            double step = (Math.abs(remaining) > MAX_SEGMENT_IN)
                    ? Math.copySign(MAX_SEGMENT_IN, remaining)
                    : remaining;
            while (opModeIsActive() && !drivetrain.runStrafeToPosition(speed, step, DistanceUnit.INCH)) {
                idle();
            }
            remaining -= step;
            if (opModeIsActive() && Math.abs(remaining) > 0.1) sleep(SEGMENT_SETTLE_MS);
        }
    }
}
