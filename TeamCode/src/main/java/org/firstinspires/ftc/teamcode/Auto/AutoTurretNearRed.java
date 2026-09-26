package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Constants.RED_GOAL_TAG_ID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Red-alliance "Near" turret autonomous. Starts against the wall to the RIGHT of the red goal, so the
 * goal is to the robot's left - the mirror of the blue variant. See {@link AutoTurretNear} for the full
 * sequence.
 */
@Autonomous(name = "Turret Near Red", group = "Red Near")
public class AutoTurretNearRed extends AutoTurretNear {

    @Override
    protected int getTargetTagId() {
        return RED_GOAL_TAG_ID; // 24
    }

    @Override
    protected double aimSeedSign() {
        return -1; // robot is right of the goal, so the goal is to the LEFT of turret center
    }
}
