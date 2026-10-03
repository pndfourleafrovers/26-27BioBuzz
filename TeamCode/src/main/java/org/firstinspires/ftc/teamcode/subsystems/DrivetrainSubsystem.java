/*
Do not edit this code for the time being. 
As time goes along, this can be edited to change all of the drive code.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * A reusable subsystem class for controlling the 4-motor drivetrain.
 * Contains initialization, encoder constants, and movement methods.
 */
public class DrivetrainSubsystem {
    /**
     * Executes a PID-based rotation using Odometry/IMU heading.
     * This method is designed to be called repeatedly in the OpMode loop() until it returns true.
     *
     * @param currentHeadingRadians The current robot heading, in radians, from pinpoint.getHeading().
     * @param targetAngleDegrees The desired absolute field angle to turn to (0 to 360).
     * @param speed The maximum power (0.0 to 1.0) allowed for the turn.
     * @return True when movement is complete (within tolerance).
     */
    private final double Kp = 0.03; // Proportional Gain: Adjusts power based on error.
    private final double MIN_POWER = 0.12; // Minimum power to overcome static friction.
    private final double TOLERANCE_MM = 15.0; // Stop when within 15mm of target.
    private final double TOLERANCE_DEGREES = 2.0;

    public boolean runOdometryTurnToAngle(double currentHeadingRadians, double targetAngleDegrees, double speed) {
        // 1. Convert current heading from radians to degrees
        double currentHeadingDegrees = Math.toDegrees(currentHeadingRadians);

        // 2. Calculate Error (Shortest path rotation)
        double error = targetAngleDegrees - currentHeadingDegrees;

        // Normalize error to the range [-180, 180] degrees (shortest path)
        while (error > 180) {
            error -= 360;
        }
        while (error <= -180) {
            error += 360;
        }

        // 3. Completion Check
        if (Math.abs(error) < TOLERANCE_DEGREES) {
            setMotorPower(0, 0, 0, 0); // Stop motors
            return true; // Turn complete
        }

        // 4. Calculate power (P-controller)
        double turnPower = error * Kp;

        // Apply speed limit
        turnPower = Math.min(speed, Math.max(-speed, turnPower));

        // Apply minimum power (prevents stalling when close to target)
        if (Math.abs(turnPower) < MIN_POWER) {
            turnPower = (turnPower > 0) ? MIN_POWER : -MIN_POWER;
        }

        // 5. Apply Power for Rotation
        // Positive error (Target > Current) = Turn Counter-Clockwise (Left)
        // With Left=FORWARD / Right=REVERSE:
        // Left needs negative power to go back, Right needs positive power to go forward.
        setMotorPower(-turnPower, -turnPower, turnPower, turnPower);

        return false; // Turn not complete
    }
    /**
     * Executes a P-based linear drive movement using Odometry X position.
     * This method assumes the robot is roughly aligned with the direction of travel (X-axis).
     *
     * @param currentPosX The current robot X position (in mm) from pinpoint.getPosX().
     * @param targetPosX The desired absolute field X position (in mm).
     * @param speed The maximum power (0.0 to 1.0) allowed for the drive.
     * @return True when movement is complete (within tolerance).
     */
    public boolean runOdoDriveX(double currentPosX, double targetPosX, double speed) {
        // 1. Calculate Error (in millimeters)
        double error = targetPosX - currentPosX;

        // 2. Completion Check
        if (Math.abs(error) < TOLERANCE_MM) {
            setMotorPower(0, 0, 0, 0); // Stop motors
            return true; // Drive complete
        }

        // 3. Calculate power (P-controller)
        double drivePower = error * Kp;

        // Apply speed limit
        drivePower = Math.min(speed, Math.max(-speed, drivePower));

        // Apply minimum power (prevents stalling when close to target)
        if (Math.abs(drivePower) < MIN_POWER) {
            drivePower = (drivePower > 0) ? MIN_POWER : -MIN_POWER;
        }

        // 4. Apply Power for Forward/Backward Drive
        // Standard Mecanum: All motors + to go forward
        setMotorPower(drivePower, drivePower, drivePower, drivePower);

        return false; // Drive not complete
    }

    /**
     * Executes a P-based linear strafe movement using Odometry Y position.
     * This method assumes the robot is roughly aligned with the Y-axis.
     *
     * @param currentPosY The current robot Y position (in mm) from pinpoint.getPosY().
     * @param targetPosY The desired absolute field Y position (in mm).
     * @param speed The maximum power (0.0 to 1.0) allowed for the drive.
     * @return True when movement is complete (within tolerance).
     */
    public boolean runOdoDriveY(double currentPosY, double targetPosY, double speed) {

        // 1. Calculate Error (in millimeters) based on Y-coordinate
        double error = targetPosY - currentPosY;

        // 2. Completion Check
        if (Math.abs(error) < TOLERANCE_MM) {
            setMotorPower(0, 0, 0, 0); // Stop motors
            return true; // Drive complete
        }

        // 3. Calculate power (P-controller)
        double drivePower = error * Kp;

        // Apply speed limit and minimum power (same as before)
        drivePower = Math.min(speed, Math.max(-speed, drivePower));
        if (Math.abs(drivePower) < MIN_POWER) {
            drivePower = (drivePower > 0) ? MIN_POWER : -MIN_POWER;
        }

        // 4. Apply Power for Strafing (Y-axis Movement)
        // Strafe Left (+Y) Standard Mecanum Vectors:
        // LF: - | RF: + | LB: + | RB: -
        setMotorPower(-drivePower, drivePower, drivePower, -drivePower);

        return false; // Drive not complete
    }

    // MARK: - Constants
    private final double WHEEL_DIAMETER_MM = 104;
    private final double ENCODER_TICKS_PER_REV = 383.6;
    private final double TRACK_WIDTH_MM = 368.5; // Distance between center of drive wheels
    private final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    private final double MOVEMENT_TIMEOUT_SECONDS = 3.0; // Safety timeout for all movements

    // MARK: - Hardware Declarations
    private DcMotorEx FrontLeft = null;
    private DcMotorEx RearLeft = null;
    private DcMotorEx FrontRight = null;
    private DcMotorEx RearRight = null;

    private ElapsedTime movementTimer = new ElapsedTime();
    private boolean movementStarted = false;
    private final double STRAFE_CORRECTION_FACTOR = 1.4;

    public void setMotorPower(double lf, double rf, double lr, double rr){
        //encoder info not needed - increases responsiveness of movements.
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setPower(lf);
        RearLeft.setPower(lr);
        FrontRight.setPower(rf);
        RearRight.setPower(rr);
    }

    /**
     * Constructor: Initializes the hardware components.
     * @param hardwareMap The hardware map object from the OpMode.
     */
    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        // Initialize hardware
        FrontLeft = hardwareMap.get(DcMotorEx.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotorEx.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotorEx.class, "Right_front");
        RearRight = hardwareMap.get(DcMotorEx.class, "Right_rear");

        // Set directions (Matching the WORKING PinpointPathAuto configuration)
        // Left is FORWARD, Right is REVERSE
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        // Configure motors
        DcMotor[] driveMotors = {FrontLeft, RearLeft, FrontRight, RearRight};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Inside DrivetrainSubsystem.java (New Method)
    public boolean runMainAim(double currentX, double currentY, double currentHeadingRadians,
                              double targetGoalX, double targetGoalY, double speed) {

        // Using placeholder constants (Kp_RADIAN, TOLERANCE_RADIANS, MIN_POWER)
        final double Kp_RADIAN = 0.5;
        final double TOLERANCE_RADIANS = Math.toRadians(2.0);
        final double MIN_POWER = 0.1;

        // 1. Calculate the required absolute angle (Bearing)
        double deltaX = targetGoalX - currentX;
        double deltaY = targetGoalY - currentY;
        double targetBearingRadians = Math.atan2(deltaY, deltaX);

        // 2. Calculate and Normalize Error
        double errorRadians = targetBearingRadians - currentHeadingRadians;

        // Normalize error to [-PI, PI] (shortest path turn)
        while (errorRadians > Math.PI) { errorRadians -= 2 * Math.PI; }
        while (errorRadians <= -Math.PI) { errorRadians += 2 * Math.PI; }

        // 3. Completion Check
        if (Math.abs(errorRadians) < TOLERANCE_RADIANS) {
            setMotorPower(0, 0, 0, 0);
            return true;
        }

        // 4. Calculate Power (P-controller)
        double turnPower = errorRadians * Kp_RADIAN;

        // Apply clamping and minimum power
        turnPower = Math.min(speed, Math.max(-speed, turnPower));
        if (Math.abs(turnPower) < MIN_POWER) {
            turnPower = (turnPower > 0) ? MIN_POWER : -MIN_POWER;
        }

        // 5. Apply Power for Rotation
        setMotorPower(-turnPower, -turnPower, turnPower, turnPower);

        return false;
    }
    /**
     * Executes an encoder-based drive or rotation movement.
     * This method is designed to be called repeatedly in the OpMode loop() until it returns true.
     *
     * @param speed Motor power (0.0 to 1.0).
     * @param distance Distance to drive (in unit). Pass 0 for rotation.
     * @param angle Angle to rotate (in unit). Pass 0 for linear drive.
     * @param unit Distance or Angle unit (e.g., DistanceUnit.INCH or AngleUnit.DEGREES).
     * @return True when movement is complete or timed out.
     */
    public boolean runDriveToPosition(double speed, double distance, double angle, Enum unit) {
        if (!movementStarted) {
            // 1. Initial configuration: Calculate targets and start the move

            double leftTargetPosition, rightTargetPosition;

            if (distance != 0) { // Linear Drive
                // Convert distance to millimeters, then to encoder ticks
                double distanceMm = (unit instanceof DistanceUnit) ? ((DistanceUnit) unit).toMm(distance) : distance;
                double targetPositionTicks = distanceMm * TICKS_PER_MM;
                leftTargetPosition = targetPositionTicks;
                rightTargetPosition = targetPositionTicks;
            } else if (angle != 0) { // Rotation
                // Calculate arc length for rotation
                double targetMm = ((AngleUnit) unit).toRadians(angle) * (TRACK_WIDTH_MM / 2);
                // CCW Rotation: Left Back, Right Forward
                leftTargetPosition = -(targetMm * TICKS_PER_MM);
                rightTargetPosition = (targetMm * TICKS_PER_MM);
            } else {
                // If both distance and angle are 0, movement is done
                return true;
            }

            // Set Target Positions
            FrontLeft.setTargetPosition((int) Math.round(leftTargetPosition));
            RearLeft.setTargetPosition((int) Math.round(leftTargetPosition));
            FrontRight.setTargetPosition((int) Math.round(rightTargetPosition));
            RearRight.setTargetPosition((int) Math.round(rightTargetPosition));

            // Set Mode and Power
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontLeft.setPower(speed);
            RearLeft.setPower(speed);
            FrontRight.setPower(speed);
            RearRight.setPower(speed);

            movementTimer.reset();
            movementStarted = true;
        }

        // 2. Completion Check
        boolean motorsBusy = FrontLeft.isBusy() || RearLeft.isBusy() || FrontRight.isBusy() || RearRight.isBusy();
        boolean timeout = movementTimer.seconds() >= MOVEMENT_TIMEOUT_SECONDS;

        if (!motorsBusy || timeout) {
            // Movement is complete or timed out, stop motors and reset flag
            stopAndResetEncoders();
            movementStarted = false;
            return true;
        }

        return false;
    }

    //Strafe Code
    public boolean runStrafeToPosition(double speed, double strafeDistance, DistanceUnit unit){
        if(!movementStarted){

            double targetTicks; //may need to declare value as 0, if there are errors.

            if(strafeDistance != 0){
                //calculate encoder ticks
                double distanceMm = unit.toMm(strafeDistance);
                targetTicks = distanceMm * TICKS_PER_MM * STRAFE_CORRECTION_FACTOR;



            } else {
                return true;
            }


            // Strafe Right (Positive Distance) Logic: FL+, FR-, RL-, RR+
            FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + (int) targetTicks);
            FrontRight.setTargetPosition(FrontRight.getCurrentPosition() - (int) targetTicks);
            RearLeft.setTargetPosition(RearLeft.getCurrentPosition() - (int) targetTicks);
            RearRight.setTargetPosition(RearRight.getCurrentPosition() + (int) targetTicks);

            // Set Mode and Power
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontLeft.setPower(speed);
            RearLeft.setPower(speed);
            FrontRight.setPower(speed);
            RearRight.setPower(speed);

            movementTimer.reset();
            movementStarted = true;
        }

        boolean motorsBusy = FrontLeft.isBusy() || RearLeft.isBusy() || FrontRight.isBusy() || RearRight.isBusy();
        boolean timeout = movementTimer.seconds() >= MOVEMENT_TIMEOUT_SECONDS;

        if (!motorsBusy || timeout) {
            // Movement is complete or timed out, stop motors and reset flag
            stopAndResetEncoders();
            movementStarted = false;
            return true;
        }

        return false;
    }
    /**
     * Stops all drive motors and resets the encoders.
     */
    public void stopAndResetEncoders() {
        FrontLeft.setPower(0);
        RearLeft.setPower(0);
        FrontRight.setPower(0);
        RearRight.setPower(0);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}