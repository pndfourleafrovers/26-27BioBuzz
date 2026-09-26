package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    public static double aimKP = 0.03;
    public static double aimDeadband = 1;
    public static double aimMinPower = 0.12;

    public static double X_POD_OFFSET = 1.25; //inches
    public static double Y_POD_OFFSET = -7; //inches

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

    // Goal AprilTag IDs the turret can lock onto.
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int RED_GOAL_TAG_ID = 24;

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

    public static double HATCH_VELOCITY = -1400; //2000, 1800, 1700, 1650
    public static double SLOW_HATCH_VELOCITY = 150;//was 10
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

    public static double redBearingOffset = 0.0;
    public static double blueBearingOffset = -2.5;

    public static double launchP = 140;//was 0
    public static double launchI = 0;
    public static double launchD = 0;
    public static double launchF = 15; //was 16

    // public static double liftUp = 0.33;
    //public static double liftDown = 0.5;

    //Blue Location
    public static double BLUE_GOAL_X = 4646;//3470.0;
    public static double BLUE_GOAL_Y = 1223;//155.0;

    //Far
    public static double BLUE_FAR_START_X = 1624;
    public static double BLUE_FAR_START_Y = 439;
    public static double BLUE_FAR_START_H = 0.0;

    public static double BLUE_FAR_SHOOT_X = 1763;
    public static double BLUE_FAR_SHOOT_Y = 367;
    public static double BLUE_FAR_SHOOT_H = 22;

    public static double BLUE_FAR_FINISH_X = 1998;
    public static double BLUE_FAR_FINISH_Y = 709;
    public static double BLUE_FAR_FINISH_H = 0;

    //Near
    public static double BLUE_NEAR_START_X = 4807;
    public static double BLUE_NEAR_START_Y = 1026;
    public static double BLUE_NEAR_START_H = 5.0;

    public static double BLUE_NEAR_SHOOT_X = 3720;//was 3564
    public static double BLUE_NEAR_SHOOT_Y = 590;//was 485
    public static double BLUE_NEAR_SHOOT_H = 45.0;

    public static double BLUE_NEAR_FINISH_X = 4297;
    public static double BLUE_NEAR_FINISH_Y = 463;
    public static double BLUE_NEAR_FINISH_H = 50;

    //Rows
    public static double BLUE_ROW1_SETUP_X = 2284;
    public static double BLUE_ROW1_SETUP_Y = 700;
    public static double BLUE_ROW1_SETUP_H = 90;

    public static double BLUE_ROW1_PICKUP_X = 2284;
    public static double BLUE_ROW1_PICKUP_Y = 1500;
    public static double BLUE_ROW1_PICKUP_H = 90;

    public static double BLUE_ROW2_SETUP_X = 2884;
    public static double BLUE_ROW2_SETUP_Y = 700;
    public static double BLUE_ROW2_SETUP_H = 90;

    public static double BLUE_ROW2_PICKUP_X = 2884;
    public static double BLUE_ROW2_PICKUP_Y = 1505;
    public static double BLUE_ROW2_PICKUP_H = 90;

    public static double BLUE_ROW3_SETUP_X = 3513;
    public static double BLUE_ROW3_SETUP_Y = 700;
    public static double BLUE_ROW3_SETUP_H = 90;

    public static double BLUE_ROW3_PICKUP_X = 3513;
    public static double BLUE_ROW3_PICKUP_Y = 1365;
    public static double BLUE_ROW3_PICKUP_H = 90;

    public static double BLUE_TURN_TO_WALL_X = 1618;
    public static double BLUE_TURN_TO_WALL_Y = 1311;
    public static double BLUE_TURN_TO_WALL_H = 90;

    //Wall
    public static double BLUE_WALL_X = 1688;
    public static double BLUE_WALL_Y = 1574;
    public static double BLUE_WALL_H = 107;


    public static double BLUE_WALL_X_TWO = 1620;
    public static double BLUE_WALL_Y_TWO = 1595;
    public static double BLUE_WALL_H_TWO = 90;

    public static double BLUE_BACK_FROM_WALL_X = 1618;
    public static double BLUE_BACK_FROM_WALL_Y = 1416;
    public static double BLUE_BACK_FROM_WALL_H = 90;

    //Gate
    public static double BLUE_GATE_SETUP_X = 3325; //was 2379, 3215
    public static double BLUE_GATE_SETUP_Y = 952; //960,
    public static double BLUE_GATE_SETUP_H = 180;

    public static double BLUE_GATE_PRESS_X = 3330; //was 3325, 3215, 3333
    public static double BLUE_GATE_PRESS_Y = 1425;
    public static double BLUE_GATE_PRESS_H = 180;


    //Red Locations
    public static double RED_GOAL_X = 4800;//3470.0;
    public static double RED_GOAL_Y = -1300;//155.0;

    //Far
    public static double RED_FAR_START_X = 1624;
    public static double RED_FAR_START_Y = -330;
    public static double RED_FAR_START_H = 0.0;

    public static double RED_FAR_SHOOT_X = 1717;
    public static double RED_FAR_SHOOT_Y = -313;
    public static double RED_FAR_SHOOT_H = -23;

    public static double RED_FAR_FINISH_X = 2119;
    public static double RED_FAR_FINISH_Y = -520;
    public static double RED_FAR_FINISH_H = 0;

    //Near
    public static double RED_NEAR_START_X = 4780;
    public static double RED_NEAR_START_Y = -1003;
    public static double RED_NEAR_START_H = -5.0;

    public static double RED_NEAR_SHOOT_X = 3676;
    public static double RED_NEAR_SHOOT_Y = -558;
    public static double RED_NEAR_SHOOT_H = -45.0;

    public static double RED_NEAR_FINISH_X = 4143;
    public static double RED_NEAR_FINISH_Y = -501;
    public static double RED_NEAR_FINISH_H = -45;

    //Rows
    public static double RED_ROW1_SETUP_X = 2314;
    public static double RED_ROW1_SETUP_Y = -700;
    public static double RED_ROW1_SETUP_H = -90;

    public static double RED_ROW1_PICKUP_X = 2314;
    public static double RED_ROW1_PICKUP_Y = -1420;
    public static double RED_ROW1_PICKUP_H = -90;

    public static double RED_ROW2_SETUP_X = 2884;
    public static double RED_ROW2_SETUP_Y = -700;
    public static double RED_ROW2_SETUP_H = -90;

    public static double RED_ROW2_PICKUP_X = 2884;
    public static double RED_ROW2_PICKUP_Y = -1420;
    public static double RED_ROW2_PICKUP_H = -90;

    public static double RED_ROW3_SETUP_X = 3509;
    public static double RED_ROW3_SETUP_Y = -607;
    public static double RED_ROW3_SETUP_H = -90;

    public static double RED_ROW3_PICKUP_X = 3491;
    public static double RED_ROW3_PICKUP_Y = -1335;
    public static double RED_ROW3_PICKUP_H = -90;

    //Wall
    public static double RED_TURN_TO_WALL_X = 1618;
    public static double RED_TURN_TO_WALL_Y = -1197;
    public static double RED_TURN_TO_WALL_H = -90;

    public static double RED_WALL_X = 1638;
    public static double RED_WALL_Y = -1500;
    public static double RED_WALL_H = -108;

    public static double RED_WALL_X_TWO = 1638;
    public static double RED_WALL_Y_TWO = -1532;
    public static double RED_WALL_H_TWO = -90;

    public static double RED_BACK_FROM_WALL_X = 1618;
    public static double RED_BACK_FROM_WALL_Y = -1336;
    public static double RED_BACK_FROM_WALL_H = -90;

    //Gate
    public static double RED_GATE_SETUP_X = 3233; //3312
    public static double RED_GATE_SETUP_Y = -869;
    public static double RED_GATE_SETUP_H = 180;

    public static double RED_GATE_PRESS_X = 3233; //3312
    public static double RED_GATE_PRESS_Y = -1379;
    public static double RED_GATE_PRESS_H = 180;
}
