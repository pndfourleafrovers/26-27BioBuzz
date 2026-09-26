package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.CLOSE_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.LAUNCHER_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchD;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchF;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchI;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.launchP;
//import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftDown;
//import static org.firstinspires.ftc.teamcode.subsystems.Constants.liftUp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LauncherSubsystemRookieBot {

    private DcMotorEx launcher;
    private DcMotorEx launchAsst;
    private DcMotorEx intake;
    private DcMotorEx hatch;
    // NOTE: The turret servos (ServoFront/ServoBack) are owned by TurretSubsystem, not here.

    public LauncherSubsystemRookieBot(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "Intake");
        hatch = hardwareMap.get(DcMotorEx.class, "Hatch");
        launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        launchAsst = hardwareMap.get(DcMotorEx.class, "LaunchAsst");

        launcher.setDirection(DcMotorEx.Direction.FORWARD);
        launchAsst.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        hatch.setDirection(DcMotor.Direction.REVERSE);



        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchAsst.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hatch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        launcher.setZeroPowerBehavior(BRAKE);
        launchAsst.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        hatch.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchAsst.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hatch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(launchP, launchI, launchD, launchF));
        launchAsst.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
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

    public void setHatchVelocity(double velocity) {
        hatch.setVelocity(velocity);
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
        hatch.setVelocity(0);
    }
}
