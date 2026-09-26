
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.FAR_LAUNCH_POWER;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.FAR_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.HATCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.INTAKE_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.LaunchPower;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.MID_LAUNCH_POWER;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.MID_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.TELE_CLOSE_LAUNCH;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.TELE_CLOSE_MIN_LAUNCH;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.TICKS_PER_REV;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;
@TeleOp(name = "TeleOpTurretSimple", group = "TeleOp")
@Configurable
@Disabled
public class TeleOpTurretSimple extends LinearOpMode {
    private DrivetrainSubsystem drivetrain = null;

    private LauncherSubsystemRookieBot launcherSubsystemRookieBot;

    //------------------------------Variables----------------------------
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private double minVelocity = TELE_CLOSE_MIN_LAUNCH;
    private boolean manualPowerOverride = true;
    private double autoLaunchPower = LaunchPower;


    //intake variables

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

        //drivetrain.stopAndResetEncoders();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        launcherSubsystemRookieBot.setHatchVelocity(0);



        while (opModeIsActive()) {

//------------------------------------------------------------GAMEPAD-1-----------------------

            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            //------------------Aims when left bumper is pressed down--------------
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

            drivetrain.setMotorPower(leftFrontPower * powerMultiplier, rightFrontPower * powerMultiplier, leftBackPower * powerMultiplier, rightBackPower * powerMultiplier);


            if (gamepad1.right_trigger > 0) {
                launcherSubsystemRookieBot.setIntakeVelocity(INTAKE_TARGET_VELOCITY);
                //outTakeTimer.reset();
            } else if (gamepad1.right_trigger == 0) {
                launcherSubsystemRookieBot.setIntakeVelocity(0);
            }

//----------------------------------------------------GAMEPAD-2-------------------------

            if (gamepad2.a) {
                launcherSubsystemRookieBot.setLauncherVelocity(1000);
            } else {
                launcherSubsystemRookieBot.setLauncherVelocity(0);
            }

            if (gamepad2.x) {
                    launcherSubsystemRookieBot.setHatchVelocity(HATCH_VELOCITY);
                } else {
                    launcherSubsystemRookieBot.setHatchVelocity(0);
                }

                double distanceFeet = -1;

/*
            if (gamepad2.left_trigger > 0) {
                    launcherSubsystemRookieBot.setLauncherVelocity(LaunchPower);
                } else {
                    launcherSubsystemRookieBot.setLauncherVelocity(0);
                }

                if (gamepad2.dpad_up) {
                    LaunchPower = FAR_LAUNCH_POWER;
                    minVelocity = FAR_MIN_VELOCITY;
                } else if (gamepad2.dpad_left) {
                    LaunchPower = MID_LAUNCH_POWER;
                    minVelocity = MID_MIN_VELOCITY;
                } else if (gamepad2.dpad_down) {
                    LaunchPower = TELE_CLOSE_LAUNCH;
                    minVelocity = TELE_CLOSE_MIN_LAUNCH;
                }
*/
                if (gamepad2.yWasPressed()) {
                    manualPowerOverride = !manualPowerOverride;
                }

//---------------------------------------------------------Telemetry-------------------------------

                double launcherVelocity = launcherSubsystemRookieBot.getLauncherVelocity();
                double launchAsstVelocity = launcherSubsystemRookieBot.getLaunchAsstVelocity();
                double intakeVelocity = launcherSubsystemRookieBot.getIntakeVelocity();
                double launcherPRM = (launcherVelocity / TICKS_PER_REV) * 60;
                double launchAsstRPM = (launchAsstVelocity / TICKS_PER_REV) * 60;
                double intakeRPM = (intakeVelocity / TICKS_PER_REV) * 60;

                telemetry.addData("-----------Launch Distance", "");
                telemetry.addData("Auto Launch Power", "%2f", autoLaunchPower);
                telemetry.addData("Manual Override", manualPowerOverride ? "ON" : "OFF");
                telemetry.addData("Launcher", "%2f", launcherVelocity);
                telemetry.addData("Launch Asst", "%2f", launchAsstVelocity);


                telemetry.addData("-----RPM----", "");
                telemetry.addData("Launcher RPM", "%2f", launcherPRM);
                telemetry.addData("Launch Asst RPM", "%2f", launchAsstRPM);
                telemetry.addData("Actual Intake Velocity", "%2f", launcherSubsystemRookieBot.getIntakeVelocity());
                telemetry.addData("Expected Intake Velocity", INTAKE_TARGET_VELOCITY);


                telemetry.update();

            }
        }
    }