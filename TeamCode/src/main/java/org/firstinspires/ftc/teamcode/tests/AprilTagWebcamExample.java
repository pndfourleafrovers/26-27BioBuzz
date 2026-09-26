/*package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebcamExample extends OpMode {
   // AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
       // aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        //update the vision portal
        //aprilTagWebcam.update();
        //AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        //aprilTagWebcam.displayDetectionTelemetry(id20);
        if (id20 != null) {
            telemetry.addData("id20 String", id20.toString());
        } else {
            telemetry.addData("id20 String", "Tag 20 not found");
        }
        telemetry.update();
    }
} */
//telemetry guide
//XYZ is x distance away, y distance away, and z distance away from the robot
//PRY is pitch, roll, and yaw in degrees
//RBE is range, bering, and elevation