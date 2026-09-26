package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemRookieBot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.subsystems.Constants.*;

import org.openftc.apriltag.AprilTagDetection;

@Disabled
public class TurretMechanismsWebcam {
    private LauncherSubsystemRookieBot launcherSubsystemRookieBot;
    public double kP = 0.0001;
    public double kD = 0.0;
    public double goalX = 0;
    public double lastError = 0;
    public double angleTolerance = 0.2;
    public final double MAX_POWER = 0.8;


    public final ElapsedTime timer = new ElapsedTime();

    private Servo ServoFront;
    private Servo ServoBack;

    private void init(HardwareMap hwmap) {
        ServoFront = hardwareMap.get(Servo.class, "ServoFront");
        ServoBack = hardwareMap.get(Servo.class, "ServoBack");
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }
/*
    public void update(AprilTagDetection curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (curID == null && curID.id == 20) {
            ServoFront.setPosition(0.5);
            ServoBack.setPosition(0.5);
            lastError = 0;
            return;
        }

        double error = goalX - curID.ftcPose.bearing;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) /deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            launcherSubsystemRookieBot.setTurret(TurretMid);
        } else {
            launcherSubsystemRookieBot.setTurret() =
        }
    }*/
}
