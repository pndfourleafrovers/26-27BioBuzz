//Do not edit code right now

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;

//odometry IMU
@Disabled
@TeleOp(name = "TeleOpPIDF", group = "TeleOp")
public class TeleOpPIDF extends LinearOpMode {


    //Declares DrivetrainSubsystem to handle motor functions
    private DrivetrainSubsystem drivetrain = null;

    //Defines Motors and pinpoint
    private DcMotorEx Intake;
    private CRServo IntakeAsst;
    private Servo Lift;
    private DcMotorEx Launcher;
    private DcMotorEx LaunchAsst;
    private GoBildaPinpointDriver pinpoint;

    //initializes variables
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private boolean outTake = false;
    private boolean test = false;
    //Same as auto
    private final double X_POD_OFFSET = -70.0;
    private final double Y_POD_OFFSET = -360.0;
    //Update if updated in auto
    private final double BLUE_GOAL_X = 155.0; //was 210
    private final double BLUE_GOAL_Y = 3470.0; //was 3370





    //----------------------------------------------------
    private double LaunchPower = 1200; //og 1500
    static final double MAX_MOTOR_VELOCITY_TICKS_PER_SEC = 2800.0;
    private double LaunchVelocity = MAX_MOTOR_VELOCITY_TICKS_PER_SEC;
    //Need to add ticks per revolution from code
    static final double TICKS_PER_REV = 28;
    private double minVelocity = 1100;  //og 1300




    @Override
    public void runOpMode() {

        drivetrain = new DrivetrainSubsystem(hardwareMap); //defines name, rotation and brake

        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        IntakeAsst = hardwareMap.get(CRServo.class, "IntakeAsst");
        Lift = hardwareMap.get(Servo.class, "Lift");
        LaunchAsst = hardwareMap.get(DcMotorEx.class, "LaunchAsst");
//        IntakeAsst = hardwareMap.get(CRServo.class, "IntakeAsst")
        //Autoaim setup
        //Initializations
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET, Y_POD_OFFSET, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //Load saved position, if available
        if (PoseStorage.currentX != 0 || PoseStorage.currentY != 0){
            pinpoint.setPosition(new Pose2D(
                DistanceUnit.MM,
                PoseStorage.currentX,
                PoseStorage.currentY,
                AngleUnit.RADIANS,
                PoseStorage.currentHeading
            ));
            telemetry.addData("Status", "Auto Loaded");
        } else{
            //defaults to normal driving if auto not used or can't be read
            pinpoint.resetPosAndIMU();
        }

        //Motor Directions
        Intake.setDirection(DcMotor.Direction.REVERSE);
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
        Launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));
        LaunchAsst.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));

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


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            //AutoAim
            //update odometry location
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();
            //Aims when left bumper is pressed down
            if (gamepad1.left_bumper) {
                double curX = pos.getX(DistanceUnit.MM);
                double curY = pos.getY(DistanceUnit.MM);
                //calculate anlge
                double targetAngleRad = Math.atan2(BLUE_GOAL_Y - curY, BLUE_GOAL_X - curX);
                double currentAngleRad = pos.getHeading(AngleUnit.RADIANS);
                //caculate error to find easiest rotation path 
                double error = Math.toDegrees(targetAngleRad - currentAngleRad);
                while (error > 180) error -= 360;
                while (error <= -180) error += 360;
                //Tuning
                //Increase if slow, decrease if wobbles
                double turnPower = error * 0.025;
                //changes yaw in this mode to allow for negatives
                yaw = -turnPower;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = -axial - lateral + yaw;
            double rightFrontPower = -axial + lateral - yaw;
            double leftBackPower = -axial + lateral + yaw;
            double rightBackPower = -axial - lateral - yaw;

            //----------------------------------DRIVING--------------------------------------------
            if (gamepad1.right_bumper) {
                if (!slowMode) {
                    slowMode = true;
                    powerMultiplier = 0.35; // was 0.3, 0.2
                }
            } else {
                if (slowMode) {
                    slowMode = false;
                    powerMultiplier = 1.0;
                }
            }

            drivetrain.setMotorPower(leftFrontPower * powerMultiplier, rightFrontPower * powerMultiplier, leftBackPower * powerMultiplier, rightBackPower * powerMultiplier);


            //Get Encoder Velocity
            //Launcher
            double launcherVelocity = Launcher.getVelocity();
            double launchAsstVelocity = LaunchAsst.getVelocity();
            //Intake
            double intakeVelocity = Intake.getVelocity();

            //Figure RPM
            //Launcher
            double launcherPRM = (launcherVelocity / TICKS_PER_REV) * 60;
            double launchAsstRPM = (launchAsstVelocity / TICKS_PER_REV) * 60;
            //Intake
            double intakeRPM = (intakeVelocity / TICKS_PER_REV) * 60;

            //Raw Speed
            //Drive
            telemetry.addData("----Motor Speed----","");
            telemetry.addData("Front Left Motor","%2f",leftFrontPower);
            telemetry.addData("Rear Left Motor","%2f",leftBackPower);
            telemetry.addData("Front Right Motor","%2f",rightFrontPower);
            telemetry.addData("Rear Right Motor","%2f",rightBackPower);
            //Launcher
            telemetry.addData("Launcher","%2f",launcherVelocity);
            telemetry.addData("Launch Asst","%2f",launchAsstVelocity);
            //Intake
            telemetry.addData("Intake","%2f",intakeVelocity);

            //RPM
            //Drive
            telemetry.addData("-----RPM----","");
            //Launcher
            telemetry.addData("Launcher RPM","%2f",launcherPRM);
            telemetry.addData("Launch Asst RPM","%2f",launchAsstRPM);
            //Intake
            telemetry.addData("Intake RPM","%2f",intakeRPM);


            telemetry.update();


            //----------------------------------------------------GAMEPAD-1-------------------------
            if (gamepad1.right_trigger > 0) {
//                test = true;
                Intake.setVelocity(-2800);
                IntakeAsst.setPower(1);
            } else {
//                outtakeAfterIntakeRelease(test); // it was a test
                Intake.setVelocity(0);
                IntakeAsst.setPower(0);
            }

            if (gamepad1.left_trigger > 0) {
                // Intake OUT
                ((DcMotorEx) Intake).setVelocity(1000);
            } else if (gamepad1.right_trigger > 0) {
                // Intake IN
                ((DcMotorEx) Intake).setVelocity(-2400);//was -2400 to test 1150rpm motor, tried 2400 dec 3rd
            } else {
                // Stop intake
                Intake.setPower(0);
            }

            //----------------------------------------------------GAMEPAD-2-------------------------
            if (gamepad2.a) {
                if (Launcher.getVelocity() >= minVelocity) {
                    Lift.setPosition(0.33);//was 0.67;

                } else {
                    Lift.setPosition(0.50);//Was 0.50
                }
            } else {
                Lift.setPosition(0.50);//was 0.50
            }

            if (gamepad2.left_trigger > 0) {
                //removed MotorEx
                (Launcher).setVelocity(LaunchPower);
                (LaunchAsst).setVelocity(-LaunchPower);
                IntakeAsst.setPower(1);
            } else {
                //removed MotorEx ((MotorEx)Launcher).setVelocity(0);
                (Launcher).setVelocity(0);
                (LaunchAsst).setVelocity(0);
                IntakeAsst.setPower(0);
            }
            if (gamepad2.dpad_up) {//FAR
                LaunchPower = 1390;//was 1500, then 1410
                minVelocity = 1370;
            } else if (gamepad2.dpad_left) {//MIDDLE DISTANCE
                LaunchPower = 1160;//Was 1225
                minVelocity = 1145;//was 1175
            } else if (gamepad2.dpad_down) {//CLOSE
                LaunchPower = 1060;//Was 1225
                minVelocity = 1045;//was 1175
            }

            if (test) {

            }
        }

    }

}



