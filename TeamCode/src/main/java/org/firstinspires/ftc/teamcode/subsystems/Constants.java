package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    public static double aimKP = 0.03;
    public static double aimDeadband = 1;
    public static double aimMinPower = 0.12;

    public static double X_POD_OFFSET = 0.75; //inches, was 1.25
    public static double Y_POD_OFFSET = -8.5; //inches, was -7

    public static double waitTime = 2.0; // This is used as the gate time in some code; it is also delay code
    public static double gateTime = 2.0;

 /*   public double kP = 0.0001;
    public double kD = 0.0;
    public double goalX = 0;
    public double lastError = 0;
    public double angleTolerance = 0.2;
    public final double MAX_POWER = 0.8;
    public double power = 0;
    public final ElapsedTime timer = new ElapsedTime();
    */
public static double turretAimKP = 0.00065;
    public static final double TurretMax = 1.0;
    public static final double TurretMid = 0.5;
    public static final double TurretMin = 0.0;
    private static final double SERVO_RANGE_DEGREES = 300.0;
    public static double turretSearchStep = 0.005;

    public static double LIFT_TIME = 0.13;
    public static double TIME_BETWEEN_SHOTS = 0.25; // was 0.2, 0.25, 0.3

    public static double LAUNCHER_TARGET_VELOCITY = 1855;
    public static double LAUNCHER_MIN_VELOCITY = 1835;

    public static double FAR_LAUNCH_POWER = 1920;//was 2040; CHANGE HERE FOR FAR LAUNCH
    public static double FAR_MIN_VELOCITY = 1900;//was 2020; MAKE THIS 20rpm LESS THAN ABOVE
    public static double MID_LAUNCH_POWER = 1550;//was 1684; CHANGE HERE FOR MID
    public static double MID_MIN_VELOCITY = 1530;//was 1664; MAKE THIS 20rpm LESS THAN ABOVE
    public static double CLOSE_LAUNCH_POWER = 1370;//was 1400
    public static double CLOSE_MIN_VELOCITY = 1350;//was 1380

    public static double TELE_CLOSE_LAUNCH = 1380;//was 1290; CHANGE HERE FOR CLOSE
    public static double TELE_CLOSE_MIN_LAUNCH = 1130;//was 1270; MAKE THIS 20rpm LESS THAN ABOVE

    public static double MAX_LAUNCH_POWER = 2000;

    //public static double HATCH_VELOCITY = -1400; //2000, 1800, 1700, 1650
    //public static double SLOW_HATCH_VELOCITY = 150;//was 10
    public static double autoLaunchMinDiff = 20;
    public static double nearestBase = 3.0;
    public static double farthestBase = 9.75;

    public static double LaunchPower = 1200;
    public static double TICKS_PER_REV = 28;

    public static double INTAKE_TARGET_VELOCITY = 2600;//was 2800
    public static double OUTTAKE_VELOCITY = -1000;
    public static double OUTTAKE_TIME = 0.6;
    public static double INTAKE_BURST_SHOT = 2800;

    public static double driveSpeed = 0.9;
    public static double driveSpeedSlow = 0.34;
    public static double rotateSpeed = 0.4;
    public static double odoSlowSpeedMultiple = 0.47;

    public static double launchP = 140;//was 0
    public static double launchI = 0;
    public static double launchD = 0;
    public static double launchF = 15; //was 16

    //Red
    public static double RedStartX = 581.5;
    public static double RedStartY = 1556.3;
    public static double RedStartH = -90;

    public static double RedForwardX = 581.5;
    public static double RedForwardY = 990.4;
    public static double RedForwardH = -90;

    public static double RedStrafeX = 2033.5;
    public static double RedStrafeY = 990.4;
    public static double RedStrafeH = -90;

    public static double RedParkX = 2033.5;
    public static double RedParkY = 1420;
    public static double RedParkH = -90;
}
