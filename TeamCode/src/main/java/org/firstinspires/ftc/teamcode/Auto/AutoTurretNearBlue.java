package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.BLUE_GOAL_TAG_ID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Blue-alliance "Near" turret autonomous. Starts against the wall to the LEFT of the blue goal, so the
 * goal is to the robot's right. See {@link AutoTurretNear} for the full sequence.
 */
@Autonomous(name = "Turret Near Blue", group = "Blue Near")
public class AutoTurretNearBlue extends AutoTurretNear {

    @Override
    protected int getTargetTagId() {
        return BLUE_GOAL_TAG_ID; // 20
    }

    @Override
    protected double aimSeedSign() {
        return +1; // robot is left of the goal, so the goal is to the RIGHT of turret center
    }
}
