package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


/*
Needs:
finalize shooting angle
update row 2 measurements - create variables as needed
figure out finish position
 */

@Autonomous(name="WorkingBlueFar", group="BlueOdo")
@Configurable
@Disabled
public class BlueOdoSubsystem extends OpMode {

    // --- DRIVE CLASS ---
    private PinpointDrive autoDrive = null;

    // Constants
    //final double LIFT_TIME = 0.14;
    //final double LAUNCHER_TARGET_VELOCITY = 1620; //1400, 1395, 1390
   // final double INTAKE_TARGET_VELOCITY = 2800;
   // final double LAUNCHER_MIN_VELOCITY = 1580; //1380, 1370, 1375
    final double TIME_BETWEEN_SHOTS = 0.4;
    final double driveSpeed = 0.75;  //was .075, 1.0
    final double driveSpeedSlow = 0.34;//was 0.5, 0.2, 0.1
    final double rotateSpeed = 0.4;

    // --- POSITIONS ---
    //Y - front-back, X - left-right, H - heading
    public static double START_X = 1390.0;  //measure from front wall out to middle of bot
    public static double START_Y = 205.0; //measure from left wall out to middle of bot
    public static double START_H = 90.0; //bot faces the back wall

    public static double SHOOT_X = 1311.0; //bot's x position for taking shots, was 1375
    public static double SHOOT_Y = 329.0; //bot's y position for taking shots, was 377

    public static double BLUE_GOAL_X = 155.0; //in mm, need to measure, was 210
    public static double BLUE_GOAL_Y = 3470.0;  //in mm, need to measure, was 3370

    public static double ROW1_SETUP_X = 840.0;
    public static double ROW1_SETUP_Y = 570.0;
    public static double ROW1_SETUP_H = 180.0; //changes to 90 degrees to the left

    public static double ROW1_PICKUP_X = 30.0; // moves forward to pick up artifacts
    public static double ROW1_PICKUP_Y = 570.0;
    public static double ROW1_PICKUP_H = 180.0;

    public static double ROW2_SETUP_X = 840.0;
    public static double ROW2_SETUP_Y = 925.0; //og 1128, then 1090 then 1036, then 1053, then 1105, then 1111, then 1000, then 950, then 940
    public static double ROW2_SETUP_H = 180.0; //changes to 90 degrees to the left

    public static double ROW2_PICKUP_X = 40.0; // moves forward to pick up artifacts
    public static double ROW2_PICKUP_Y = 925.0; //og 1036, then 1090 then og, then 1053, then 1105, then 1111, then 1000, then 950, then 940
    public static double ROW2_PICKUP_H = 180.0;

    public static double FINISH_X = 1256.0;
    public static double FINISH_Y = 216.0;
    public static double FINISH_H = 90.0;

    private GoBildaPinpointDriver pinpoint = null;
    private final double X_POD_OFFSET = 60.0; // double check measurements
    private final double Y_POD_OFFSET = -184.0; //double check measurements

    int shotsToFire = 3;
    // private ElapsedTime launchTimer = new ElapsedTime();
   //  private ElapsedTime feederTimer = new ElapsedTime();

   // private DcMotorEx Launcher = null;
    private DcMotorEx LaunchAsst = null;
    //private DcMotorEx Intake = null;
   // private Servo Lift = null;
    private CRServo IntakeAsst = null;

    private enum AutoState {
        SHOOT_ALIGN,
        PREP_SHOT,
        LAUNCHER_SPINUP,
        FIRE_SHOT,
        WAIT_FOR_FEEDER,

        ROW1_TURN_TO_ARTIFACTS,
        ROW1_DRIVE_TO_ARTIFACTS,
        ROW1_RETURN_TO_SHOOT,

        ROW2_TURN_TO_ARTIFACTS,
        ROW2_DRIVE_TO_ARTIFACTS,
        ROW2_RETURN_TO_SHOOT,

        FINISH
    }

    private BlueOdoSubsystem.AutoState autoState;
    private BlueOdoSubsystem.AutoState returnState;


    @Override
    public void init(){
        // Pinpoint Setup
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setOffsets(X_POD_OFFSET,Y_POD_OFFSET,DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // Set Start Position
        pinpoint.setPosX(START_X, DistanceUnit.MM);
        pinpoint.setPosY(START_Y, DistanceUnit.MM);
        pinpoint.setHeading(START_H, AngleUnit.DEGREES);

        // Initialize Drive
        autoDrive = new PinpointDrive(hardwareMap);
//---------------------STARTING STEP IN CODE---------------
        autoState = AutoState.SHOOT_ALIGN ;

        // Hardware Init
       /* Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        IntakeAsst = hardwareMap.get(CRServo.class, "IntakeAsst");
        Lift = hardwareMap.get(Servo.class, "Lift");
        Launcher = hardwareMap.get(DcMotorEx.class,"Launcher");
        LaunchAsst = hardwareMap.get(DcMotorEx.class,"LaunchAsst");

        Launcher.setDirection(DcMotorEx.Direction.FORWARD);
        LaunchAsst.setDirection(DcMotorEx.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LaunchAsst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Launcher.setZeroPowerBehavior(BRAKE);
        LaunchAsst.setZeroPowerBehavior(BRAKE);
        Intake.setZeroPowerBehavior(BRAKE);
        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LaunchAsst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(150,0,0,21));
        LaunchAsst.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(150,0,0,21));
     */ }

    @Override
    public void init_loop(){
        //Lift.setPosition(0.5);
        pinpoint.update();



        //check to make sure bot odometry is set. We can use this for measurement and check to make sure odometry is working correctly.
        Pose2D pos = pinpoint.getPosition();
        telemetry.addData("Status", "Initialized - PUSH TO TEST");
        telemetry.addData("X (mm)", "%.1f", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pos.getY(DistanceUnit.MM));
        telemetry.addData("H (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start(){
       // Intake.setVelocity(2100);
    }

    @Override
    public void loop(){
        pinpoint.update();
        //current position
        double curX = pinpoint.getPosX(DistanceUnit.MM);
        double curY = pinpoint.getPosY(DistanceUnit.MM);
        double curH = pinpoint.getHeading(AngleUnit.DEGREES);

        switch (autoState) {

            case SHOOT_ALIGN:
                if (shootingPosition(curX, curY, curH)) {
                    shotsToFire = 3;
                    returnState = AutoState.ROW1_TURN_TO_ARTIFACTS;
                    autoState = AutoState.PREP_SHOT;
                }
                break;

            /* case PREP_SHOT:
                Launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                LaunchAsst.setVelocity(LAUNCHER_TARGET_VELOCITY);
                IntakeAsst.setPower(1.0);
                launchTimer.reset();
                autoState = AutoState.ROW1_TURN_TO_ARTIFACTS;
                break;

            case LAUNCHER_SPINUP:
                if(Launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY && launchTimer.seconds() > 0.5){
                    autoState = AutoState.FIRE_SHOT;
                }
                break;
            case FIRE_SHOT:
                Lift.setPosition(0.33);
                feederTimer.reset();
                autoState = AutoState.WAIT_FOR_FEEDER;
                break;

            case WAIT_FOR_FEEDER:
                if(feederTimer.seconds() > LIFT_TIME){
                    Lift.setPosition(0.5);
                    shotsToFire--;

                    if (shotsToFire > 0) {
                        autoState = AutoState.PREP_SHOT;
                    } else {
                        autoDrive.stop();
                        Intake.setVelocity(0);
                        IntakeAsst.setPower(0);
                        autoState = returnState;
                    }
                }
                break;

//------------------------------------------ROW ONE--------------------------------------
            */case ROW1_TURN_TO_ARTIFACTS:
                //keeps X/Y, rotates to artifacts
                if (autoDrive.runToPosition(curX, curY, curH, ROW1_SETUP_X, ROW1_SETUP_Y, ROW1_SETUP_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW1_DRIVE_TO_ARTIFACTS;
                }
                break;

            case ROW1_DRIVE_TO_ARTIFACTS:
               // Intake.setVelocity(INTAKE_TARGET_VELOCITY);
                // moves forward to pick up artifacts
                if (autoDrive.runToPosition(curX, curY, curH, ROW1_PICKUP_X, ROW1_PICKUP_Y, ROW1_PICKUP_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW1_RETURN_TO_SHOOT;
                }
                break;

            case ROW1_RETURN_TO_SHOOT:
                // takes in current positions, change to new coordinates, sets drive speed and rotation speed
               // Intake.setVelocity(0);
                if (shootingPosition(curX, curY, curH)) {
                shotsToFire = 3;
                 returnState = AutoState.ROW2_TURN_TO_ARTIFACTS;
                    autoState = AutoState.ROW2_TURN_TO_ARTIFACTS;
                }
                break;


//-------------------------------------------------ROW TWO----------------------------
            case ROW2_TURN_TO_ARTIFACTS:
                //keeps X/Y, rotates to artifacts
                if (autoDrive.runToPosition(curX, curY, curH, ROW2_SETUP_X, ROW2_SETUP_Y, ROW2_SETUP_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW2_DRIVE_TO_ARTIFACTS;
                }
                break;
            case ROW2_DRIVE_TO_ARTIFACTS:
                // Intake.setVelocity(INTAKE_TARGET_VELOCITY);
                // moves forward to pick up artifacts
                if (autoDrive.runToPosition(curX, curY, curH, ROW2_PICKUP_X, ROW2_PICKUP_Y, ROW2_PICKUP_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW2_RETURN_TO_SHOOT;
                }
                break;
            case ROW2_RETURN_TO_SHOOT:
                // Intake.setVelocity(0);
                if (shootingPosition(curX, curY, curH)) {
                    shotsToFire = 3;
                    returnState = AutoState.FINISH;
                    autoState = AutoState.FINISH;
                }
                break;



            case FINISH:
                if (autoDrive.runToPosition(curX, curY, curH, FINISH_X, FINISH_Y, FINISH_H, driveSpeed, rotateSpeed)) {
                    autoDrive.stop();
                    PoseStorage.currentX = pinpoint.getPosX(DistanceUnit.MM);
                    PoseStorage.currentY = pinpoint.getPosY(DistanceUnit.MM);
                    PoseStorage.currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                }
                break;
        }

        telemetry.addData("AutoState", autoState);
       // telemetry.addData("Shots Left", shotsToFire);
        telemetry.addData("X", "%.1f", curX);
        telemetry.addData("Y", "%.1f", curY);
        telemetry.addData("H", "%.1f", curH);
        telemetry.update();
    }

    private boolean shootingPosition(double curX, double curY, double curH){
        double autoMainAim = autoDrive.getHeadingToGoal(curX, curY, BLUE_GOAL_X, BLUE_GOAL_Y);

        return autoDrive.runToPosition(curX, curY, curH, SHOOT_X, SHOOT_Y, autoMainAim, driveSpeed, rotateSpeed);
    }

    @Override
    public void stop(){
    }
}
