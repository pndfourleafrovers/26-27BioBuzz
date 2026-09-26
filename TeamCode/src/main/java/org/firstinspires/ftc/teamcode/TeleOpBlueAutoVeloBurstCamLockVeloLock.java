//Do not edit code right now

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//odometry IMU
@TeleOp(name = "TeleOp", group = "TeleOp")
@Disabled
//@Configurable
public class TeleOpBlueAutoVeloBurstCamLockVeloLock extends LinearOpMode {
    //Declares DrivetrainSubsystem to handle motor functions
    private DrivetrainSubsystem drivetrain = null;
    // private AprilTagWebcam aprilTagWebcam;
    //private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private LauncherSubsystemRookieBot launcherSubsystemRookieBot;

    //------------------------------Variables----------------------------
    //initializes variables
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private double minVelocity = TELE_CLOSE_MIN_LAUNCH;
    private boolean manualPowerOverride = true;
    private double autoLaunchPower = LaunchPower;


    //intake variables
    //private ElapsedTime outTakeTimer = new ElapsedTime();

    private int burstShotsRemaining = 0;
    private ElapsedTime burstLaunchTimer = new ElapsedTime();
    private ElapsedTime burstFeederTimer = new ElapsedTime();
    private ElapsedTime hatchTimer = new ElapsedTime();
    private boolean burstFeeding = false;

    private boolean intakeAsstToggle = true;

    private double[] bearingHistory = new double[5];
    private int bearingIndex = 0;

    private double[] veloHistory = new double[5];
    private int veloIndex = 0;


    @Override
    public void runOpMode() {

        drivetrain = new DrivetrainSubsystem(hardwareMap); //defines name, rotation and brake
        launcherSubsystemRookieBot = new LauncherSubsystemRookieBot(hardwareMap);

        //aprilTagWebcam = new AprilTagWebcam();
        //aprilTagWebcam.init(hardwareMap, telemetry);

        //Start Encoder
        //Drive
        drivetrain.stopAndResetEncoders();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //panelsTelemetry.update();
        //for (int i = 0; i < veloHistory.length; i++) veloHistory[i] = nearestBase * 30.48;
        waitForStart();
        launcherSubsystemRookieBot.setHatchVelocity(0);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //launcherSubsystem.setIntakeAsstPower(1.0);
            //launcherSubsystem.setIntakeAsstTwoPower(1.0);

//------------------------------------------------------------GAMEPAD-1-----------------------

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            //AutoAim
            //update odometry location
            // aprilTagWebcam.update();
            //AprilTagDetection targetTag = aprilTagWebcam.getTagBySpecificId(20); //blue = 20,red = 24


            //------------------Aims when left bumper is pressed down--------------
            /*if (gamepad1.left_bumper) {
                if (targetTag != null && targetTag.ftcPose != null) {
                    bearingHistory[bearingIndex] = targetTag.ftcPose.bearing;
                    bearingIndex = (bearingIndex + 1) % 5;

                    double smoothedBearing = 0;
                    for (int i = 0; i < 5; i++) {
                        smoothedBearing += bearingHistory[i];
                    }
                    smoothedBearing /= 5;


                    double bearingError = smoothedBearing - blueBearingOffset;

                    if (Math.abs(bearingError) < aimDeadband) {
                        yaw = 0;
                    } else {
                        yaw = aimKP * bearingError;
                        if (Math.abs(yaw) < aimMinPower) {
                            yaw = Math.signum(yaw) * aimMinPower;
                        }
                    }

                    telemetry.addData("Locked", "ID: %d, Bearing Error: %.2f",
                            targetTag.id, bearingError);

                } else {
                    telemetry.addData("Not Locked", "Searching for Blue Goal");
                }
            }

           */
            //Tuning Bearing
            /*
            if (gamepad1.dpadRightWasPressed()) {
                bearingOffset += 0.05;}

            if (gamepad1.dpadLeftWasPressed())  {
               bearingOffset -= 0.05;
            }
*/
            /*
            //If tuning does not work, activate this code and comment the code out above.
                        if (gamepad1.dpad_right) {
                bearingOffset += 0.05;
            }

            if (gamepad1.dpad_left)  {
                bearingOffset -= 0.05;
            }
             */

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            //New code for adding vectors
            double forward = -axial;
            double strafe = -lateral;
            double turn = yaw;

            double maxDriveSpeed = 1.0;

            double driveMagnitude = Math.hypot(forward, strafe);

            if (driveMagnitude > maxDriveSpeed) {
                forward = (forward / driveMagnitude) * maxDriveSpeed;
                strafe = (strafe / driveMagnitude) * maxDriveSpeed;
            }

            double leftFrontPower = forward - strafe - turn;
            double rightFrontPower = forward + strafe + turn;
            double leftBackPower = forward + strafe - turn;
            //change to - turn if robot is spinning strangely or drifts
            double rightBackPower = forward - strafe + turn;

            double max = Math.max(1.0, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;

           /* if (gamepad1.left_bumper) {
                if (!slowMode) {
                    slowMode = true;
                    powerMultiplier = odoSlowSpeedMultiple; // was 0.3, 0.2
                }
            } else {
                if (slowMode) {
                    slowMode = false;
                    powerMultiplier = 1.0;
                }
            } */

            drivetrain.setMotorPower(leftFrontPower * powerMultiplier, rightFrontPower * powerMultiplier, leftBackPower * powerMultiplier, rightBackPower * powerMultiplier);

           /* if (gamepad1.aWasPressed()){
                launcherSubsystemRookieBot.setHatchVelocity(intakeAsstToggle ? 1.0:0.0);
                intakeAsstToggle = !intakeAsstToggle;
            } */

            if (gamepad1.right_trigger > 0) {
                launcherSubsystemRookieBot.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                //outTakeTimer.reset();
            } else if (gamepad1.right_trigger == 0) {
                launcherSubsystemRookieBot.setIntakeVelocity(0);
            }
            /*
            if (gamepad1.right_bumper == true) {
                launcherSubsystemRookieBot.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
            }
            */


            // else if (gamepad1.left_trigger > 0) {
            // Intake Out
            //launcherSubsystemRookieBot.setIntakeVelocity(OUTTAKE_VELOCITY);
            //IntakeAsst.setPower(0);
            //}

           /* else if (burstShotsRemaining > 0) {
                launcherSubsystemRookieBot.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
            } else if (outTakeTimer.seconds() < OUTTAKE_TIME) {
                launcherSubsystemRookieBot.setIntakeVelocity(OUTTAKE_VELOCITY);
            } else {
                // Stop everything
                launcherSubsystemRookieBot.setIntakeVelocity(0);
                //IntakeAsst.setPower(0);
            } */

//----------------------------------------------------GAMEPAD-2-------------------------
           /*  if(gamepad2.aWasPressed()){
                burstShotsRemaining = 3;
                burstLaunchTimer.reset();
                burstFeeding = false;
                launcherSubsystem.setIntakeAsstPower(1.0);
                intakeAsstToggle = true;
                launcherSubsystem.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
            }

            if(burstShotsRemaining > 0){
                if(!burstFeeding){
                    if(launcherSubsystem.getLauncherVelocity() >= minVelocity && burstLaunchTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launcherSubsystem.setLauncherVelocity(autoLaunchPower);
                        launcherSubsystem.liftUp();
                        burstFeederTimer.reset();
                        burstFeeding = true;
                    }
                } else{
                    if (burstFeederTimer.seconds()>LIFT_TIME){
                        launcherSubsystem.liftDown();
                        burstShotsRemaining--;
                        burstLaunchTimer.reset();
                        burstFeeding = false;
                        if(burstShotsRemaining == 0){
                            launcherSubsystem.setIntakeVelocity(0);
                        }
                    }
                }
            } */
/*
            if (gamepad2.x) {
                hatchTimer.reset();
                launcherSubsystemRookieBot.setHatchVelocity(HATCH_VELOCITY);
            } else if (hatchTimer.milliseconds() > 2000) {
                launcherSubsystemRookieBot.setHatchVelocity(0);
            }
*/
            if (gamepad2.x) {
                launcherSubsystemRookieBot.setHatchVelocity(HATCH_VELOCITY);
            } else {
                launcherSubsystemRookieBot.setHatchVelocity(0);
            }

            double distanceFeet = -1;

          /*  if (gamepad2.left_trigger > 0 && burstShotsRemaining == 0) {
                //looks for april tag, smooths range reading, converts to feet
                if (targetTag != null && targetTag.ftcPose != null) {
                    veloHistory[veloIndex] = targetTag.ftcPose.range;
                    veloIndex = (veloIndex + 1) % 5;
                    double smoothedRange = 0;
                    for (int i = 0; i < 5; i++) smoothedRange += veloHistory[i];
                    smoothedRange /= 5;
                    distanceFeet = smoothedRange / 30.48;
                }
                //If distance - finds percentage of distance and then scales the power by adding to close(base)
                autoLaunchPower = (!manualPowerOverride && distanceFeet >= 0) ? TELE_CLOSE_LAUNCH + (distanceFeet - nearestBase) / (farthestBase - nearestBase) * (FAR_LAUNCH_POWER - TELE_CLOSE_LAUNCH) : LaunchPower;
*/

                /*if (!manualPowerOverride && distanceFeet >= 0) {
                    double t = Math.sqrt((distanceFeet - nearestBase) / (farthestBase - nearestBase));
                    autoLaunchPower = TELE_CLOSE_LAUNCH + t * (FAR_LAUNCH_POWER - TELE_CLOSE_LAUNCH);
                } else {
                    autoLaunchPower = LaunchPower;
                }
                 */


            //compares calculated power with max power. sets to lower number


            //  double t = Math.sqrt((distanceFeet - nearestBase) / (farthestBase - nearestBase));
            //  autoLaunchPower = TELE_CLOSE_LAUNCH + t * (FAR_LAUNCH_POWER - TELE_CLOSE_LAUNCH);

           /* } else {
                if (burstShotsRemaining == 0) {
                    launcherSubsystemRookieBot.setLauncherVelocity(0);
                }
            } */

            if (gamepad2.left_trigger > 0) {
                launcherSubsystemRookieBot.setLauncherVelocity(LaunchPower);
            } else {
                launcherSubsystemRookieBot.setLauncherVelocity(0);
            }

            if (gamepad2.dpad_up) {//FAR
                LaunchPower = FAR_LAUNCH_POWER;//was 1500, then 1410, 1300
                minVelocity = FAR_MIN_VELOCITY;
            } else if (gamepad2.dpad_left) {//MIDDLE DISTANCE
                LaunchPower = MID_LAUNCH_POWER;//Was 1225, 1160
                minVelocity = MID_MIN_VELOCITY;//was 1175
            } else if (gamepad2.dpad_down) {//CLOSE
                LaunchPower = TELE_CLOSE_LAUNCH;//Was 1225
                minVelocity = TELE_CLOSE_MIN_LAUNCH;//was 1175, 1060
            }

            if (gamepad2.yWasPressed()) {
                manualPowerOverride = !manualPowerOverride;
            }
//---------------------------------------------------------Telemetry-------------------------------
            //Figure RPM
            //Launcher
            double launcherVelocity = launcherSubsystemRookieBot.getLauncherVelocity();
            double launchAsstVelocity = launcherSubsystemRookieBot.getLaunchAsstVelocity();
            double intakeVelocity = launcherSubsystemRookieBot.getIntakeVelocity();
            //Launcher
            double launcherPRM = (launcherVelocity / TICKS_PER_REV) * 60;
            double launchAsstRPM = (launchAsstVelocity / TICKS_PER_REV) * 60;
            //Intake
            double intakeRPM = (intakeVelocity / TICKS_PER_REV) * 60;

            //Raw Speed
            //Drive
            //Launcher
            telemetry.addData("-----------Launch Distance", "");
            //telemetry.addData("Goal Distance Feet","%2f",distanceFeet == -1 ? 0.0:distanceFeet);
            telemetry.addData("Auto Launch Power", "%2f", autoLaunchPower);
            telemetry.addData("Manual Override", manualPowerOverride ? "ON" : "OFF");
            telemetry.addData("Launcher", "%2f", launcherVelocity);
            telemetry.addData("Launch Asst", "%2f", launchAsstVelocity);
            //panelsTelemetry.addData("Launcher Velocity", launcherVelocity);
            //panelsTelemetry.addData("Launch Asst Velocity", launchAsstVelocity);
            //Intake


            //RPM
            //Drive
            telemetry.addData("-----RPM----", "");
            //Launcher
            telemetry.addData("Launcher RPM", "%2f", launcherPRM);
            telemetry.addData("Launch Asst RPM", "%2f", launchAsstRPM);
            //Intake
            telemetry.addData("Actual Intake Velocity", "%2f", launcherSubsystemRookieBot.getIntakeVelocity());
            telemetry.addData("Expected Intake Velocity", INTAKE_TARGET_VELOCITY);


            telemetry.update();
            //panelsTelemetry.update();

            //aprilTagWebcam.stop();

        }
    }
}