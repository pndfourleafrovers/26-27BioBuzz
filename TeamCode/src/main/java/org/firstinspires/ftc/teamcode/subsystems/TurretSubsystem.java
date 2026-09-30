/*
Reusable subsystem for the turret, which rides on two positional servos (ServoFront + ServoBack)
ganged on ONE axis. Any op mode builds this during init; constructing it drives the turret to its
midpoint (150 deg) so it starts from a known, centered position.

The servos are configured for 300 deg of travel, so setPosition(0.0..1.0) maps to 0..300 deg, and the
midpoint 150 deg = position 0.50.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
public class TurretSubsystem {

    /** Result of a closed-loop aim step. */
    public enum AimStatus { LOCKED, TRACKING, AT_LIMIT }

    // MARK: - Constants
    // Servos are configured for 300 deg of travel, so position 0.0..1.0 => 0..300 deg.
    public static final double SERVO_RANGE_DEGREES = 300.0;
    // Midpoint: 0.5 * 300 = 150 deg.
    public static final double CENTER_POSITION = 0.5;

    // MARK: - Hardware
    private Servo servoFront = null;
    private Servo servoBack = null;

    // Commanded turret position, owned here so closed-loop aiming persists across loops.
    private double turretPos = Constants.TurretMid;

    /**
     * Constructor: grabs both turret servos and immediately centers the turret.
     * Because op modes build their subsystems during init (before waitForStart), constructing this
     * subsystem is what makes the turret initialize to its 150 deg midpoint on startup.
     *
     * @param hardwareMap The hardware map object from the op mode.
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        servoFront = hardwareMap.get(Servo.class, "ServoFront");
        servoBack = hardwareMap.get(Servo.class, "ServoBack");

        // Initialize to the midpoint (150 deg).
        centerTurret();
    }

    /**
     * Drives the turret to its midpoint (150 deg).
     */
    public void centerTurret() {
        // NOTE: If tuning shows the two servos don't physically align at 0.50/0.50, replace this with
        // per-servo center constants (e.g. CENTER_FRONT / CENTER_BACK) and set each servo separately.
        turretPos = Constants.TurretMid;
        setTurretPosition(turretPos);
    }

    /**
     * Closed-loop aim step: nudges the turret to drive a camera angle error toward zero.
     * Call this repeatedly (once per loop) while a target is visible. Because the camera is mounted on
     * the turret, nulling the error self-corrects without needing the servo-to-turret gear ratio.
     *
     * Owns the accumulator, gain (Constants.turretAimKP), deadband (Constants.aimDeadband), clamp, and
     * reach-limit reporting.
     *
     * @param errorDeg Horizontal angle to the target in degrees (e.g. Limelight tx, or a webcam bearing
     *                 with its offset already subtracted). 0 = centered.
     * @return LOCKED if within the deadband, AT_LIMIT if the turret is pinned at its travel edge, else TRACKING.
     */
    public AimStatus aimAtAngle(double errorDeg) {
        if (Math.abs(errorDeg) <= Constants.aimDeadband) {
            return AimStatus.LOCKED;
        }
        // Flip the sign of this term if the turret chases the wrong way.
        turretPos -= Constants.turretAimKP * errorDeg;
        // Anti-windup: clamp the accumulator to the turret's travel range.
        turretPos = Math.max(Constants.TurretMin, Math.min(Constants.TurretMax, turretPos));
        setTurretPosition(turretPos);

        boolean atLimit = (turretPos <= Constants.TurretMin || turretPos >= Constants.TurretMax);
        return atLimit ? AimStatus.AT_LIMIT : AimStatus.TRACKING;
    }

    /** @return The turret's current commanded position (0.0 to 1.0). */
    public double getCommandedPosition() {
        return turretPos;
    }

    /**
     * Manually move the turret by a position delta (used for searching when the target is not visible).
     * Moves the same accumulator the aim loop uses, so re-acquiring the target resumes smoothly. Clamps
     * to the turret's travel range.
     *
     * @param deltaPos Change in servo position this call (sign should match aimAtAngle's convention).
     */
    public void nudge(double deltaPos) {
        turretPos += deltaPos;
        turretPos = Math.max(Constants.TurretMin, Math.min(Constants.TurretMax, turretPos));
        setTurretPosition(turretPos);
    }

    /**
     * Commands the turret axis to a position. Both ganged servos receive the same value.
     * The value is clamped to the valid [0.0, 1.0] servo range.
     *
     * @param position Target servo position (0.0 = 0 deg, 0.5 = 150 deg, 1.0 = 300 deg).
     */
    public void setTurretPosition(double position) {
        double clamped = Math.max(0.0, Math.min(1.0, position));
        servoFront.setPosition(clamped);
        servoBack.setPosition(clamped);
    }

    /** @return The last commanded position of the front servo (0.0 to 1.0). */
    public double getFrontPosition() {
        return servoFront.getPosition();
    }

    /** @return The last commanded position of the back servo (0.0 to 1.0). */
    public double getBackPosition() {
        return servoBack.getPosition();
    }

    /**
     * Converts a servo position to its approximate angle in degrees, for readable telemetry.
     *
     * @param position Servo position (0.0 to 1.0).
     * @return Approximate angle in degrees (0 to 300).
     */
    public double positionToDegrees(double position) {
        return position * SERVO_RANGE_DEGREES;
    }
}
