/*

package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class StallOutCode {
  /*Logic:
    Every .5 seconds, the bot checks if it is stuck. If it is, it moves on to the next state.
    This is done by checking if the bot has moved less than 5mm in x and y, and less than 2 degrees in heading.
    If it has, it is considered stuck and moves on to the next state.
    This is done in the checkStuck method, which takes the next state as a parameter.
    The checkStuck method is called in every state that involves movement, just before the movement code.
    */

/*
    //place this code just before
    //@Override
    //public void init() {
    private ElapsedTime stuckTimer = new ElapsedTime();

    public double lastX = 0;
    public double lastY = 0;
    public double lastH = 0;

    private void checkStuck(AutoState nextState){
        if(stuckTimer.seconds() > 0.5){
            double x = pinpoint.getPosX(DistanceUnit.MM);
            double y = pinpoint.getPosY(DistanceUnit.MM);
            double h = pinpoint.getHeading(AngleUnit.DEGREES);
            if (Math.abs(x - lastX) < 5 && Math.abs(y - lastY) < 5 && Math.abs(h - lastH) < 2){
                autoState = nextState;
            }
            lastX = x;
            lastY = y;
            lastH = h;
            stuckTimer.reset();
        }
    }

/*This line goes in code that has movement. If the bot is sitting still, do not add.
As you add this, it should call in the next autoState that is defined in that case.
The code can be placed after the if statement and before break. As an example:

            case ROW1_TURN_TO_ARTIFACTS:
                if (autoDrive.runToPosition(curX, curY, curH, row1SetupX, row1SetupY, row1SetupH, driveSpeed, rotateSpeed)) {
                    autoState = AutoState.ROW1_DRIVE_TO_ARTIFACTS;
                }
 ADD HERE>>>>   checkStuck(AutoState.ROW1_DRIVE_TO_ARTIFACTS);
                break;



Cases to add:
Shoot_Align
                checkStuck(AutoState.SHOOT);

Row 1 turn to artifacts
                checkStuck(AutoState.ROW1_DRIVE_TO_ARTIFACTS);

Row 1 drive to artifacts
                checkStuck(AutoState.SHOOT_ALIGN);

Row 2 turn to artifacts
                checkStuck(AutoState.ROW2_DRIVE_TO_ARTIFACTS);

Row 2 drive to artifacts
                checkStuck(AutoState.SHOOT_ALIGN);
        OR      checkStuck(AutoState.ROW2_BACK);

Row 3 turn to artifacts
                checkStuck(AutoState.ROW3_DRIVE_TO_ARTIFACTS);

Row 3 drive to artifacts
                checkStuck(AutoState.SHOOT_ALIGN);

Turn to wall
                checkStuck(AutoState.BACK_UP_FROM_WALL);

Back up from wall
                checkStuck(AutoState.DRIVE_TO_WALL_TWO);

Go to Gate
                checkStuck(AutoState.OPEN_GATE);

*/
