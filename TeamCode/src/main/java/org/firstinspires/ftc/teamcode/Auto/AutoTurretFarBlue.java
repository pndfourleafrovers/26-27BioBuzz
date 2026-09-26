package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.BLUE_GOAL_TAG_ID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Blue-alliance "Far" turret autonomous. Starts against the far wall from the blue goal, facing it.
 * See {@link AutoTurretFar} for the full sequence.
 */
@Autonomous(name = "Turret Far Blue", group = "Blue Far")
public class AutoTurretFarBlue extends AutoTurretFar {

    @Override
    protected int getTargetTagId() {
        return BLUE_GOAL_TAG_ID; // 20
    }

    @Override
    protected double lateralSign() {
        return +1;
    }

    @Override
    protected double aimSeedSign() {
        return +1; // goal is to the right of the turret's centered direction from the blue far start pose
    }
}
