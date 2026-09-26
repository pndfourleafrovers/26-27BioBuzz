//Do not edit code right now

package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//odometry IMU
@TeleOp(name = "TEST Blue TeleOp", group = "TeleOp")
@Configurable
@Disabled

public class TESTTeleOpBlueCamAim extends LinearOpMode {
    //Declares DrivetrainSubsystem to handle motor functions
    private DrivetrainSubsystem drivetrain = null;
    //private AprilTagWebcam aprilTagWebcam;

    //Defines Motors
    private DcMotorEx Intake;
    private CRServo IntakeAsst;
    private Servo Lift;
    private DcMotorEx Launcher;
    private DcMotorEx LaunchAsst;
//------------------------------Variables----------------------------
    // auto shoot variables
    int shotsToFire = 0;
    long lastShotTime = 0;
    public static long TIME_BETWEEN_SHOTS = 300;

    //initializes variables
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    public static double bearingOffset = -1.5;

    //intake variables
    private final double INTAKE_VELOCITY = 2800;
    private final double OUTTAKE_VELOCITY = -1000;
    public static double OUTTAKE_TIME = 0.25;
    private ElapsedTime outTakeTimer = new ElapsedTime();

    //Shooting Variables
    public static double LAUNCH_SERVO_UPPIES = 0.33;
    public static double LAUNCH_SERVO_DOWNERS = 0.5;
    public static double FAR_LAUNCH_POWER = 1680;//was 1680
    public static double FAR_MIN_VELOCITY = 1640;
    public static double MID_LAUNCH_POWER = 1360;//was1320
    public static double MID_MIN_VELOCITY = 1320;
    public static double CLOSE_LAUNCH_POWER = 1200;
    public static double CLOSE_MIN_VELOCITY = 1150;
    private double LaunchPower = 1200; //og 1500


    //Need to add ticks per revolution from code
    static final double TICKS_PER_REV = 28;
    private double minVelocity = 1100;  //og 1300

    private final double slowSpeedMultiple = 0.45;

    private boolean intakeAsstToggle = true;


    @Override
    public void runOpMode() {

        drivetrain = new DrivetrainSubsystem(hardwareMap); //defines name, rotation and brake

        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        IntakeAsst = hardwareMap.get(CRServo.class, "IntakeAsst");
        Lift = hardwareMap.get(Servo.class, "Lift");
        LaunchAsst = hardwareMap.get(DcMotorEx.class, "LaunchAsst");

        //aprilTagWebcam = new AprilTagWebcam();
        //aprilTagWebcam.init(hardwareMap, telemetry);


        //Motor Directions
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        LaunchAsst.setDirection(DcMotor.Direction.FORWARD);

        //Start Encoder
        //Drive
        drivetrain.stopAndResetEncoders();
        //Launcher
        Launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LaunchAsst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Intake
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Use encoders to find velocity
        //Launcher
        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LaunchAsst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Intake
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //PID with Launch motors
        Launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(150,0,0,15.3));
        LaunchAsst.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(150,0,0,15.3));

        //Brake when motor at zero
        //Launcher
        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LaunchAsst.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Intake
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


//------------------------------------------------------------GAMEPAD-1-----------------------

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            //AutoAim
            //update odometry location
            //aprilTagWebcam.update();
            //------------------Aims when left bumper is pressed down--------------
            /*if (gamepad1.left_bumper) {
              //  AprilTagDetection targetTag = aprilTagWebcam.getTagBySpecificId(20); //blue = 20,red = 24

                if (targetTag != null && targetTag.ftcPose != null) {
                    double bearing = targetTag.ftcPose.bearing;

                    double kP = 0.03;
                    yaw = kP * (bearing - bearingOffset);  //make -bearing if turning opposite

                    telemetry.addData("Locked", "ID: %d,Target Bearing: %.2f, Offset: %.2f",
                            targetTag.id, bearing, bearingOffset);

                } else {
                    telemetry.addData("Not Locked", "Searching for Blue Goal");
                }

            } */
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

            if (gamepad1.right_bumper) {
                if (!slowMode) {
                    slowMode = true;
                    powerMultiplier = slowSpeedMultiple; // was 0.3, 0.2
                }
            } else {
                if (slowMode) {
                    slowMode = false;
                    powerMultiplier = 1.0;
                }
            }

            drivetrain.setMotorPower(leftFrontPower * powerMultiplier, rightFrontPower * powerMultiplier, leftBackPower * powerMultiplier, rightBackPower * powerMultiplier);

            if (gamepad1.aWasPressed()){
                IntakeAsst.setPower(intakeAsstToggle ? 1.0:0.0);
                intakeAsstToggle = !intakeAsstToggle;
            }



            if (gamepad1.right_trigger > 0) {
                Intake.setVelocity(INTAKE_VELOCITY);
                //IntakeAsst.setPower(1);
                outTakeTimer.reset();
            }
            else if (gamepad1.left_trigger > 0) {
                // Intake Out
                Intake.setVelocity(OUTTAKE_VELOCITY);//was -2400 to test 1150rpm motor, tried 2400 dec 3rd
                //IntakeAsst.setPower(0);
            }

            else if (outTakeTimer.seconds() < OUTTAKE_TIME) {
                    Intake.setVelocity(OUTTAKE_VELOCITY);
                } else {
                // Stop everything
                Intake.setVelocity(0);
                //IntakeAsst.setPower(0);
            }

//----------------------------------------------------GAMEPAD-2-------------------------
            /*if (gamepad2.a  && shotsToFire == 0) {
                if (Launcher.getVelocity() >= minVelocity) {
                    shotsToFire = 3;

                    if (shotsToFire > 0) {
                        long currentTime = System.currentTimeMillis();

                        if (Launcher.getVelocity() >= minVelocity) {
                            if (currentTime - lastShotTime >= TIME_BETWEEN_SHOTS) {
                                if (Lift.getPosition() == LAUNCH_SERVO_DOWNERS) {
                                    Lift.setPosition(LAUNCH_SERVO_UPPIES);
                                } else {
                                    Lift.setPosition(LAUNCH_SERVO_DOWNERS);
                                    shotsToFire--;
                                }
                                lastShotTime = currentTime;
                            }
                        } else {
                            Lift.setPosition(LAUNCH_SERVO_DOWNERS);
                            shotsToFire = 0;
                        }
                    } else {
                        Lift.setPosition(LAUNCH_SERVO_DOWNERS);
                    }
                }
            } */

            // Start shooting sequence when A is pressed (only if not already shooting)
            if (gamepad2.a && shotsToFire == 0 && Launcher.getVelocity() >= minVelocity) {
                shotsToFire = 3;
                lastShotTime = System.currentTimeMillis();
                Lift.setPosition(LAUNCH_SERVO_UPPIES);  // Fire first shot immediately
            }

            // Continue shooting sequence (runs every loop until all shots fired)
            if (shotsToFire > 0) {
                long currentTime = System.currentTimeMillis();

                if (Launcher.getVelocity() >= minVelocity) {
                    if (currentTime - lastShotTime >= TIME_BETWEEN_SHOTS) {
                        // Toggle lift position
                        if (Lift.getPosition() < 0.5) {  // Currently up (UPPIES = 0.33) was 0.4
                            Lift.setPosition(LAUNCH_SERVO_DOWNERS);  // Come back down
                            shotsToFire--;
                        } else {  // Currently down (DOWNERS = 0.5)
                            Lift.setPosition(LAUNCH_SERVO_UPPIES);  // Go up for next shot
                        }
                        lastShotTime = currentTime;
                    }
                } else {
                    // Launcher slowed down, abort shooting
                    Lift.setPosition(LAUNCH_SERVO_DOWNERS);
                    shotsToFire = 0;
                }
            }


            if (gamepad2.left_trigger > 0) {
                //removed MotorEx
                (Launcher).setVelocity(LaunchPower);
                (LaunchAsst).setVelocity(-LaunchPower);
                //IntakeAsst.setPower(1);
            } else {
                //removed MotorEx ((MotorEx)Launcher).setVelocity(0);
                (Launcher).setVelocity(0);
                (LaunchAsst).setVelocity(0);
                //IntakeAsst.setPower(0);
            }
            if (gamepad2.dpad_up) {//FAR
                LaunchPower = FAR_LAUNCH_POWER;//was 1500, then 1410, 1300
                minVelocity = FAR_MIN_VELOCITY;
            } else if (gamepad2.dpad_left) {//MIDDLE DISTANCE
                LaunchPower = MID_LAUNCH_POWER;//Was 1225, 1160
                minVelocity = MID_MIN_VELOCITY;//was 1175
            } else if (gamepad2.dpad_down) {//CLOSE
                LaunchPower = CLOSE_LAUNCH_POWER;//Was 1225
                minVelocity = CLOSE_MIN_VELOCITY;//was 1175, 1060
            }
//---------------------------------------------------------Telemetry-------------------------------
            //Figure RPM
            //Launcher
            double launcherVelocity = Launcher.getVelocity();
            double launchAsstVelocity = LaunchAsst.getVelocity();
            //Intake
            double intakeVelocity = Intake.getVelocity();
            //Launcher
            double launcherPRM = (launcherVelocity / TICKS_PER_REV) * 60;
            double launchAsstRPM = (launchAsstVelocity / TICKS_PER_REV) * 60;
            //Intake
            double intakeRPM = (intakeVelocity / TICKS_PER_REV) * 60;

            //Raw Speed
            //Drive
            telemetry.addData("----Motor Speed----","");
            //Launcher
            telemetry.addData("Launcher","%2f",launcherVelocity);
            telemetry.addData("Launch Asst","%2f",launchAsstVelocity);
            //Intake


            //RPM
            //Drive
            telemetry.addData("-----RPM----","");
            //Launcher
            telemetry.addData("Launcher RPM","%2f",launcherPRM);
            telemetry.addData("Launch Asst RPM","%2f",launchAsstRPM);
            //Intake
            telemetry.addData("Actual Intake Velocity","%2f",Intake.getVelocity());
            telemetry.addData("Expected Intake Velocity",INTAKE_VELOCITY);


            telemetry.update();
        }
        //aprilTagWebcam.stop();

    }


}