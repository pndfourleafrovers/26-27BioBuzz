package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.RED_GOAL_TAG_ID;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Red-alliance camera turret teleop. Locks the turret onto the red goal AprilTag (ID 24).
 * All behavior lives in {@link TeleOpTurretCamera}.
 */
@TeleOp(name = "TeleOpTurretCamera Red", group = "TeleOp")
@Configurable
public class TeleOpTurretCameraRed extends TeleOpTurretCamera {
    @Override
    protected int getTargetTagId() {
        return RED_GOAL_TAG_ID;
    }
}
