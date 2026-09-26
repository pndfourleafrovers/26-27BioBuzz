package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;
import org.firstinspires.ftc.teamcode.subsystems.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

/*
----------SHOOT 6----------
Shoot
Pick up 1
shoot
 */

@Autonomous(name="Red Far 1", group="Red Far")
@Disabled


public class RedFarOneLine extends OpMode {

    private PinpointDrive autoDrive = null;
    private GoBildaPinpointDriver pinpoint = null;

    int shotsToFire = 3;
    private ElapsedTime launchTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime waitTimer = new ElapsedTime();
    private ElapsedTime outTakeTimer = new ElapsedTime();
    private ElapsedTime stuckTimer = new ElapsedTime();

    public double lastX = 0;
    public double lastY = 0;
    public double lastH = 0;

    private LauncherSubsystemRookieBot launcherSubsystem;

    private enum AutoState {
        POWER_UP,
        SHOOT_ALIGN,
        SHOOT,
        SHOT_COUNT,

        ROW1_TURN_TO_ARTIFACTS,
        ROW1_DRIVE_TO_ARTIFACTS,
        OUTTAKE_TIMER,

        ROW2_TURN_TO_ARTIFACTS,
        ROW2_DRIVE_TO_ARTIFACTS,

        ROW3_TURN_TO_ARTIFACTS,
        ROW3_DRIVE_TO_ARTIFACTS,

        TURN_TO_WALL,
        DRIVE_TO_WALL,
        BACK_UP_FROM_WALL,
        DRIVE_TO_WALL_TWO,

        GO_TO_GATE,
        OPEN_GATE,
        HOLD_GATE,

        WAIT,

        FINISH
    }

    private AutoState autoState;
    private AutoState returnState;
    //private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private void checkStuck(AutoState nextState){
        if(stuckTimer.seconds() > 0.5){
            double x = pinpoint.getPosX(DistanceUnit.MM);
            double y = pinpoint.getPosY(DistanceUnit.MM);
            double h = pinpoint.getHeading(AngleUnit.DEGREES);
            if (Math.abs(x - lastX) < 5 && Math.abs(y - lastY) < 5 && Math.abs(h - lastH) < 2){
                autoState = nextState;
            }
            lastX = x;
            lastY = y;
            lastH = h;
            stuckTimer.reset();
        }
    }

    @Override
    public void init(){
        autoDrive = new PinpointDrive(hardwareMap);
        //panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getDEFAULT_FTC());

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET,Y_POD_OFFSET,DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.recalibrateIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, RED_FAR_START_X, RED_FAR_START_Y, AngleUnit.DEGREES, RED_FAR_START_H));
        pinpoint.update();

        autoState = AutoState.POWER_UP;
        returnState = AutoState.ROW1_TURN_TO_ARTIFACTS;

        launcherSubsystem = new LauncherSubsystemRookieBot(hardwareMap);
    }

    @Override
    public void init_loop(){
        //launcherSubsystem.setLiftPosition(0.5);
        pinpoint.update();

        Pose2D pos = pinpoint.getPosition();
        double goalHeading = autoDrive.getHeadingToGoal(pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), RED_GOAL_X, RED_GOAL_Y);
        telemetry.addData("Status", "Initialized - PUSH TO TEST");
        telemetry.addData("X (mm) Front-Back", "%.1f", pos.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm) Left-Right", "%.1f", pos.getY(DistanceUnit.MM));
        telemetry.addData("H (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Goal Heading", "%.1f", goalHeading);
        telemetry.update();
    }

    @Override
    public void start(){
        launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
        //launcherSubsystem.setIntakeAsstPower(1.0);
        waitTimer.reset();
        stuckTimer.reset();
    }



    @Override
    public void loop(){
        pinpoint.update();
        double rawX = pinpoint.getPosX(DistanceUnit.MM);
        double rawY = pinpoint.getPosY(DistanceUnit.MM);
        // Scale X and Y to compensate for encoder resolution mismatches
        double curX = pinpoint.getPosX(DistanceUnit.MM);
        double curY = pinpoint.getPosY(DistanceUnit.MM);
        double curH = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);


        //-------------------------------SHOOTING------------------------
        if (autoState == AutoState.SHOOT_ALIGN) {
            double targetHeading = autoDrive.getHeadingToGoal(curX, curY, RED_GOAL_X, RED_GOAL_Y);
            //telemetry.addData("DEBUG Target Heading", "%.1f", targetHeading);
            //telemetry.addData("DEBUG Current H", "%.1f", curH);
        }

        switch (autoState) {
            case POWER_UP:
                launcherSubsystem.setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
                autoState = AutoState.SHOOT_ALIGN;
                break;

            case SHOOT_ALIGN:
                if (autoDrive.runToPosition(curX,curY,curH,RED_FAR_SHOOT_X,RED_FAR_SHOOT_Y,RED_FAR_SHOOT_H, driveSpeedSlow, rotateSpeed)){
                    shotsToFire = 3;
                    launcherSubsystem.setLauncherVelocity(LAUNCHER_TARGET_VELOCITY);
                    //launcherSubsystem.setIntakeAsstPower(1.0);
                    launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                    launchTimer.reset();
                    autoState = AutoState.SHOOT;
                }
                checkStuck(AutoState.SHOOT);
                break;

            case SHOOT:
                if(launcherSubsystem.isLauncherReady() && launchTimer.seconds() > TIME_BETWEEN_SHOTS){
                    //launcherSubsystem.liftUp();
                    feederTimer.reset();
                    launcherSubsystem.setHatchVelocity(HATCH_VELOCITY);
                    autoState = AutoState.SHOT_COUNT;
                }
                break;

            case SHOT_COUNT:
                if(feederTimer.seconds() > LIFT_TIME){
                    //launcherSubsystem.liftDown();
                    shotsToFire--;

                    if (shotsToFire > 0) {
                        launchTimer.reset(); //add if launcher does not work correctly
                        autoState = AutoState.SHOOT;
                    } else {
                        autoDrive.stop();
                        launcherSubsystem.setIntakeVelocity(0);
                        launcherSubsystem.setHatchVelocity(0);
                        autoState = returnState;
                    }
                }
                break;


                //--------------------------------ROW PICKUP---------------------
            case ROW1_TURN_TO_ARTIFACTS:
                if (autoDrive.runToPosition(curX, curY, curH, RED_ROW1_SETUP_X, RED_ROW1_SETUP_Y, RED_ROW1_SETUP_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW1_DRIVE_TO_ARTIFACTS;
                }
                checkStuck(AutoState.ROW1_DRIVE_TO_ARTIFACTS);
                break;


            case ROW1_DRIVE_TO_ARTIFACTS:
                launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                if (autoDrive.runToPosition(curX, curY, curH, RED_ROW1_PICKUP_X, RED_ROW1_PICKUP_Y, RED_ROW1_PICKUP_H, driveSpeed, rotateSpeed)) {
                    returnState = AutoState.FINISH;
                    autoState = AutoState.OUTTAKE_TIMER;
                    outTakeTimer.reset();
                }
                checkStuck(AutoState.OUTTAKE_TIMER);
                break;

            case OUTTAKE_TIMER:
                if (outTakeTimer.seconds() < OUTTAKE_TIME) {
                    launcherSubsystem.setIntakeVelocity(OUTTAKE_VELOCITY);
                } else {
                    // Stop everything
                    launcherSubsystem.setIntakeVelocity(0);
                    //IntakeAsst.setPower(0);
                    autoState = AutoState.SHOOT_ALIGN;
                }
                break;

            case ROW2_TURN_TO_ARTIFACTS:
                if (autoDrive.runToPosition(curX, curY, curH, RED_ROW2_SETUP_X, RED_ROW2_SETUP_Y, RED_ROW2_SETUP_H, driveSpeed, rotateSpeed)) {

                    autoState = AutoState.ROW2_DRIVE_TO_ARTIFACTS;
                }
                break;

            case ROW2_DRIVE_TO_ARTIFACTS:
                launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                if (autoDrive.runToPosition(curX, curY, curH, RED_ROW2_PICKUP_X, RED_ROW2_PICKUP_Y, RED_ROW2_PICKUP_H, driveSpeed, rotateSpeed)) {
                    returnState = AutoState.ROW3_TURN_TO_ARTIFACTS;
                    autoState = AutoState.SHOOT_ALIGN;
                    outTakeTimer.reset();
                }
                break;

            case ROW3_TURN_TO_ARTIFACTS:
                if (autoDrive.runToPosition(curX, curY, curH, RED_ROW3_SETUP_X, RED_ROW3_SETUP_Y, RED_ROW3_SETUP_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW3_DRIVE_TO_ARTIFACTS;
                }
                break;

            case ROW3_DRIVE_TO_ARTIFACTS:
                launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                if (autoDrive.runToPosition(curX, curY, curH, RED_ROW3_PICKUP_X, RED_ROW3_PICKUP_Y, RED_ROW3_PICKUP_H, driveSpeed, rotateSpeed)) {
                    returnState = AutoState.FINISH;
                    autoState = AutoState.SHOOT_ALIGN;
                    outTakeTimer.reset();
                }
                break;



                //----------------------------------WALL---------------------
            case TURN_TO_WALL:
                if (autoDrive.runToPosition(curX, curY, curH, RED_TURN_TO_WALL_X, RED_TURN_TO_WALL_Y, RED_TURN_TO_WALL_H, driveSpeed, rotateSpeed)) {
                    launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                    autoState = AutoState.DRIVE_TO_WALL;
                }
                break;

            case DRIVE_TO_WALL:
                if (autoDrive.runToPosition(curX, curY, curH, RED_WALL_X, RED_WALL_Y, RED_WALL_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.BACK_UP_FROM_WALL;
                }
                break;

            case BACK_UP_FROM_WALL:
                if (autoDrive.runToPosition(curX, curY, curH, RED_BACK_FROM_WALL_X, RED_BACK_FROM_WALL_Y, RED_BACK_FROM_WALL_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.DRIVE_TO_WALL_TWO;
                }
                break;

            case DRIVE_TO_WALL_TWO:
                if (autoDrive.runToPosition(curX, curY, curH, RED_WALL_X, RED_WALL_Y, RED_WALL_H, driveSpeed, rotateSpeed)) {
                    returnState = AutoState.FINISH;
                    autoState = AutoState.SHOOT_ALIGN;
                }
                break;

                //--------------------------------GATE------------------------
            case GO_TO_GATE:
                if (autoDrive.runToPosition(curX, curY, curH, RED_GATE_SETUP_X, RED_GATE_SETUP_Y, RED_GATE_PRESS_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.OPEN_GATE;
                }
                break;

            case OPEN_GATE:
                if (autoDrive.runToPosition(curX, curY, curH, RED_GATE_PRESS_X, RED_GATE_PRESS_Y, RED_GATE_PRESS_H, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.SHOOT_ALIGN;
                }
                break;
            case HOLD_GATE:
                if(waitTimer.seconds()> waitTime){
                    returnState = AutoState.FINISH;
                    autoState = AutoState.SHOOT_ALIGN;
                }
                break;

                //---------------------------------PAUSE---------------------
            case WAIT:
                if(waitTimer.seconds()> waitTime){
                    autoState = returnState;
                }
                break;

                //------------------------------END-------------------------
            case FINISH:
                if (autoDrive.runToPosition(curX, curY, curH, RED_FAR_FINISH_X, RED_FAR_FINISH_Y, RED_FAR_FINISH_H, driveSpeed, rotateSpeed)) {
                    autoDrive.stop();
                    PoseStorage.currentX = curX;   // Use scaled X value
                    PoseStorage.currentY = curY;  // Use scaled Y value
                    PoseStorage.currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                }
                break;
        }

        // Debug: show errors and commands
        double xErr = RED_FAR_SHOOT_X - curX;
        double yErr = RED_FAR_SHOOT_Y - curY;
        double hErr = RED_FAR_SHOOT_H - Math.toDegrees(curH);
        while (hErr > 180) hErr -= 360;
        while (hErr <= -180) hErr += 360;

        telemetry.addData("AutoState", autoState);
        telemetry.addData("X", "%.1f → %.1f (err: %.1f)", curX, RED_FAR_SHOOT_X, xErr);
        telemetry.addData("Y (scaled)", "%.1f → %.1f (err: %.1f)", curY, RED_FAR_SHOOT_Y, yErr);
        telemetry.addData("X (scaled)", "%.1f → %.1f (err: %.1f)", curX, RED_FAR_SHOOT_X, xErr);

        telemetry.addData("Y (raw)", "%.1f", rawY);
        telemetry.addData("H", "%.1f → %.1f (err: %.1f)", Math.toDegrees(curH), RED_FAR_SHOOT_H, hErr);
        telemetry.update();
    }


    @Override
    public void stop(){
    }
}
