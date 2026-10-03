package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PinpointDrivePP {

    private DcMotorEx FrontLeft, RearLeft, FrontRight, RearRight;

    //PID Variables
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastTime = 0;
    private double lastErrorX = 0;
    private double lastErrorY = 0;
    private double lastErrorH = 0;

    //PID Tuning
    public static double DRIVE_P = 0.00071; // 0.0015, 0.0017, 0.0020, 0.0015, 0.0017, 0.0018, 0.001755 applies power based on distance - increase until bot moves past target
    public static double DRIVE_D = 0.00008; //was 0.002, 0.0001, 0.0002, 0.00009, 0.00005 applies brakes to slow down approach - helps prevent wobbling - increase if overshoots, decrease if vibrates

    public static double STRAFE_P = 0.0012; //
    public static double STRAFE_D = 0.0001; //

    public static double TURN_P = 0.00054; //0.02 applies power based on distance
    public static double TURN_D = 0.00023; //0.01 applies brakes to slow down approach - helps prevent wobbling



    //Odometry Tuning
    //increases over changes
    //private final double DRIVE_GAIN = 0.02; //may need tuned was 0.03, 0.02 - increases speed over time when going to a point
    //private final double TURN_GAIN  = 0.01; //may need tuned for rotations and strafing was 0.02 - increases speed over time when rotating

    private final double TOLERANCE_MM = 5.0;  //was 15.0, 20.0 allows for slight variation. can be tuned, if needed
    private final double TOLERANCE_DEGREES = 2.0; //allows for slight variation. can be tuned, if needed

    private final double MIN_POWER = 0.15; // keeps bot moving slowly when near target - should allow for more accurate movements. Increase if short, decrease if movement isn't fluid


    public PinpointDrivePP(HardwareMap hardwareMap) {
        //declares motors
        FrontLeft = hardwareMap.get(DcMotorEx.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotorEx.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotorEx.class, "Right_front");
        RearRight = hardwareMap.get(DcMotorEx.class, "Right_rear");



        // Directions: Left = Forward, Right = Reverse
        // based on bot 2
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        //ensures motors turn off when powered off
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // *** FIX: Ensure motors are in a RUN mode ***
        DcMotor[] driveMotors = {FrontLeft, RearLeft, FrontRight, RearRight};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Motors will NOT move without this!
        }
    }

    /**
     * Drives to a specific field position (X, Y, Heading)
     * used for driving motion in auto code
     */
    public boolean runToPosition(double currentX, double currentY, double currentHeadingDeg,
                                 double targetX, double targetY, double targetHeadingDeg,
                                 double maxDriveSpeed, double maxTurnSpeed) {

        double xError = targetX - currentX; //finds difference of target and current x for moving logic
        double yError = targetY - currentY; //finds difference of target and current y for moving logic

        //double currentHeadingDeg = Math.toDegrees(currentHeadingRad); //converts current heading to degrees
        double headingError = targetHeadingDeg - currentHeadingDeg; //finds difference of target and current heading measurements
        while (headingError > 180)  headingError -= 360; //allows bot to turn left or right to get the correct heading the quickest
        while (headingError <= -180) headingError += 360; //allows bot to turn left or right to get the correct heading the quickest

        //Math.abs - absolute value to find distance between target and current position
        //if less than set tolerance, movement ceases
        if (Math.abs(xError) < TOLERANCE_MM &&
                Math.abs(yError) < TOLERANCE_MM &&
                Math.abs(headingError) < TOLERANCE_DEGREES) {
            stop();
            return true;
        }

        //Start timer for PD tuning
        double currentTime = pidTimer.seconds();
        double dt = currentTime - lastTime;

        if (dt < 0.005) dt = 0.005; //prevents divide by zero error
        lastTime = currentTime;

        // converts measurements to field-centric
        double currentHeadingRad = Math.toRadians(currentHeadingDeg);
        double sin = Math.sin(currentHeadingRad);
        double cos = Math.cos(currentHeadingRad);

        //calculate Proportional (P)
        //makes field errors (distance to endpoint) bot-centric
        //double pForward = (xError * cos) + (yError * sin);
        //double pStrafe  = (-xError * sin) + (yError * cos);
        double pForward = (yError * cos) + (xError * sin);
        double pStrafe  = (-yError * sin) + (xError * cos);
        double pTurn    = headingError;

        //calculate Derivative (D)
        //change in error to determine when to slow
        double dX = (xError - lastErrorX) / dt;
        double dY = (yError - lastErrorY) / dt;
        double dH = (headingError - lastErrorH) / dt;

        //change field velocities to make them field centric
        //double dForward = (dX * cos) + (dY * sin);
        //double dStrafe  = (-dX * sin) + (dY * cos);
        double dForward = (dY * cos) + (dX * sin);
        double dStrafe  = (-dY * sin) + (dX * cos);
        double dTurn    = dH;

        //Updates last error
        lastErrorX = xError;
        lastErrorY = yError;
        lastErrorH = headingError;

        //movements using P, D - allows for speed change depending on distance to point
        double forward = (pForward * DRIVE_P) + (dForward * DRIVE_D);
        double strafe  = (pStrafe * STRAFE_P) + (dStrafe * STRAFE_D);
        double turn = (pTurn * TURN_P) + (dTurn * TURN_D);




        //applies conversion to movements (old)
        //double forward = (xError * cos) + (yError * sin);
        //double strafe  = (-xError * sin) + (yError * cos);



        // Apply Minimum Power (Friction Feedforward) - should ease movements when near target position
        // This ensures the robot doesn't stall when close to the target
        // If the bot is not at the correct position (> 0.01), then the bot should slowly move to the correct
        if (Math.abs(forward) < MIN_POWER && Math.abs(forward) > 0.01)
            forward = Math.signum(forward) * MIN_POWER;
        if (Math.abs(strafe) < MIN_POWER && Math.abs(strafe) > 0.01)
            strafe = Math.signum(strafe) * MIN_POWER;
        if (Math.abs(turn) < MIN_POWER && Math.abs(turn) > 0.01)
            turn = Math.signum(turn) * MIN_POWER;


        // Limits speed to decrease overshoot
        //math.max - chooses the higher number
        //math.min - chooses the lower number
        // gain increases speed based on distance. This code allows for control
        forward = Math.max(-maxDriveSpeed, Math.min(maxDriveSpeed, forward));
        strafe  = Math.max(-maxDriveSpeed, Math.min(maxDriveSpeed, strafe));
        turn    = Math.max(-maxTurnSpeed,  Math.min(maxTurnSpeed,  turn));

        //new code for vectors
        double driveMagnatude = Math.hypot(forward, strafe);

        if (driveMagnatude > maxDriveSpeed) {
            forward = (forward / driveMagnatude) * maxDriveSpeed;
            strafe = (strafe / driveMagnatude) * maxDriveSpeed;
        }

        // Mecanum Mixing (Left=Fwd, Right=Rev logic)
        double lf = forward - strafe - turn;
        double rf = forward + strafe + turn;
        double lr = forward + strafe - turn;
        double rr = forward - strafe + turn;

        /*possible mixing, if needed
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lr = forward - strafe + turn;
        double rr = forward + strafe - turn;
         */

        //normalize motors to not exceed 1.0
        double max = Math.max(1.0, Math.abs(lf));
        max = Math.max(max, Math.abs(rf));
        max = Math.max(max, Math.abs(lr));
        max = Math.max(max, Math.abs(rr));

        lf /= max;
        rf /= max;
        lr /= max;
        rr /= max;

        FrontLeft.setPower(lf);
        RearLeft.setPower(lr);
        FrontRight.setPower(rf);
        RearRight.setPower(rr);

        return false;
    }
    /*
            //old code
            double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr))));
            if (max > 1.0) {
                lf /= max; rf /= max; lr /= max; rr /= max;
            }

            FrontLeft.setPower(lf);
            RearLeft.setPower(lr);
            FrontRight.setPower(rf);
            RearRight.setPower(rr);

            return false;
        }

     */
    //Get heading to goal
    public double getHeadingToGoal(double currentX, double currentY, double targetX, double targetY) {
        double dy = targetY - currentY; //difference of target and current y
        double dx = targetX - currentX; // difference of target and current x
        double headingToGoal = Math.atan2(dy, dx); // calculates angle into radians
        return Math.toDegrees(headingToGoal); //converts radians to degrees
    }


    public void stop() {
        FrontLeft.setPower(0);
        RearLeft.setPower(0);
        FrontRight.setPower(0);
        RearRight.setPower(0);
    }
}
