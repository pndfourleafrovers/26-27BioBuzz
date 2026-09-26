/*package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherSubsystem {

    private DcMotorEx launcher;
    private DcMotorEx launchAsst;
    private DcMotorEx intake;
    private Servo lift;
    private CRServo intakeAsst;
    private CRServo intakeAsstTwo;

    public LauncherSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeAsst = hardwareMap.get(CRServo.class, "IntakeAsst");
        intakeAsstTwo = hardwareMap.get(CRServo.class, "IntakeAsstTwo");
        lift = hardwareMap.get(Servo.class, "Lift");
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        launchAsst = hardwareMap.get(DcMotorEx.class, "LaunchAsst");

        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launchAsst.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);


        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchAsst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher.setZeroPowerBehavior(BRAKE);
        launchAsst.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchAsst.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launchP, launchI, launchD, launchF));
        launchAsst.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launchP, launchI, launchD, launchF));
    }

    public void setLauncherVelocity(double velocity) {
        launcher.setVelocity(velocity);
        launchAsst.setVelocity(velocity);
    }

    public double getLauncherVelocity() {
        return launcher.getVelocity();
    }

    public boolean isLauncherReady() {
        return launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY;
    }
    public boolean isCloseLauncherReady() {
        return launcher.getVelocity() >= CLOSE_MIN_VELOCITY;
    }
    public void setIntakeVelocity(double velocity) {
        intake.setVelocity(velocity);
    }

    public void setIntakeAsstPower(double power) {
        intakeAsst.setPower(power);
        intakeAsstTwo.setPower(-power);
    }

    public void setIntakeAsstTwoPower(double power){
        intakeAsstTwo.setPower(power);
    }

    public void setLiftPosition(double position) {
        lift.setPosition(position);
    }

    public void liftUp() {
        lift.setPosition(liftUp);
    }

    //public void liftDown() {
        lift.setPosition(liftDown);
    }

    public double getIntakeVelocity() {
        return intake.getVelocity();
    }

    public double getLaunchAsstVelocity() {
        return launchAsst.getVelocity();
    }



    public void stopAll() {
        launcher.setVelocity(0);
        launchAsst.setVelocity(0);
        intake.setVelocity(0);
        intakeAsst.setPower(0);
        intakeAsstTwo.setPower(0);
    }
} */
