package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.RED_GOAL_TAG_ID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Red-alliance "Far" turret autonomous. Starts against the far wall from the red goal, facing it.
 * See {@link AutoTurretFar} for the full sequence. The red field is the mirror of blue, so the strafe
 * sign is flipped; verify the strafe direction on the field and adjust if needed.
 */
@Autonomous(name = "Turret Far Red", group = "Red Far")
public class AutoTurretFarRed extends AutoTurretFar {

    @Override
    protected int getTargetTagId() {
        return RED_GOAL_TAG_ID; // 24
    }

    @Override
    protected double lateralSign() {
        return -1;
    }

    @Override
    protected double aimSeedSign() {
        return -1; // goal is to the left of the turret's centered direction from the red far start pose (mirrored)
    }
}
