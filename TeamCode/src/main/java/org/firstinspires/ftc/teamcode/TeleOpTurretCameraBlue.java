package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.BLUE_GOAL_TAG_ID;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Blue-alliance camera turret teleop. Locks the turret onto the blue goal AprilTag (ID 20).
 * All behavior lives in {@link TeleOpTurretCamera}.
 */
@TeleOp(name = "TeleOpTurretCamera Blue", group = "TeleOp")
@Configurable
public class TeleOpTurretCameraBlue extends TeleOpTurretCamera {
    @Override
    protected int getTargetTagId() {
        return BLUE_GOAL_TAG_ID;
    }
}
