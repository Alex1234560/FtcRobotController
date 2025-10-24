package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AprilTagVision; // <-- IMPORT YOUR NEW CLASS


import com.qualcomm.robotcore.eventloop.opmode.Disabled; // Remove this too
import com.qualcomm.robotcore.hardware.Gamepad;

//@Disabled //Remove If You wAnt this to show up on the Driver Station


@TeleOp
public class LearningToCodeRobot extends OpMode {
    private Drive robotDrive;
    private AprilTagVision vision;




    @Override
    public void init() {
        telemetry.addData("HELLO","I DID IT");

        robotDrive = new Drive(hardwareMap);

        vision = new AprilTagVision(hardwareMap, "Webcam 1");

    }

    @Override
    public void loop() {
        vision.update();

        robotDrive.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, (.5+(gamepad1.right_trigger/2)));

        if (vision.isTagVisible()) {
            // A tag is visible! Get the data from our helper methods.
            double range = vision.getRange();
            double bearing = vision.getBearing();
            double yaw = vision.getYaw();
            int tagID = vision.getID();

            // --- ADD THESE TWO LINES ---
            double xOffset = vision.getX(); // Get the new X value
            double yOffset = vision.getY(); // Get the new Y value

            telemetry.addData("Status", "Tag Detected!");
            telemetry.addData("Tag ID", tagID);
            telemetry.addData("Range", "%.1f in", range);
            telemetry.addData("Bearing", "%.1f deg", bearing);
            telemetry.addData("Yaw", "%.1f deg", yaw);


            // --- AND ADD THESE TWO LINES ---
            telemetry.addData("X Offset", "%.1f in", xOffset);
            telemetry.addData("Y Offset", "%.1f in", yOffset);

        } else {
            // --- TAG IS NOT VISIBLE ---
            // Display a different status message. This will overwrite the old data.
            telemetry.addData("Tag = ", "N/A");
        }

        // IMPORTANT: The telemetry.update() call should be outside the if/else block
        // so the screen refreshes every single loop, regardless of whether a tag is seen.
        telemetry.update(); // This was missing, but is crucial for OpMode



    }
}



