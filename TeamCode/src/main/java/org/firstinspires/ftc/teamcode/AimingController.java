package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// This is NOT an OpMode. It's a helper class to calculate aiming movements.
public class AimingController {

    private final AprilTagVision vision;
    private final Telemetry telemetry;

    // --- Tuning Constants ---
    // You can adjust these values to change how the robot behaves.
    // Increase a GAIN value to make the robot react more aggressively.
    // Decrease it to make the robot react more smoothly.


    // --- Target Values ---
    // These define the "perfect" position relative to the AprilTag.
    private static final double BEARING_TOLERANCE = 2; // Target horizontal offset from tag center (inches). 0 means centered.
    private static final double X_TOLERANCE = 2; // Target horizontal offset from tag center (inches). 0 means centered.
    private static final double TARGET_X_OFFSET = -6; // Target horizontal offset from tag center (inches). 0 means centered.
    private static final double TARGET_RANGE = 40.0;   // Target distance from tag (inches).
    private static final double TARGET_TOLERANCE = 2;   // Target distance from tag (inches).

    /**
     * Constructor: This is called when you create an AimingController object.
     */
    public AimingController(AprilTagVision vision, Telemetry telemetry) {
        this.vision = vision;
        this.telemetry = telemetry;
    }

    /**
     * This is the single function you'll call from TeleOp or Autonomous.
     * It checks for a tag and returns the calculated motor powers needed to aim.
     *
     * @return A double array containing [axial, lateral, yaw] powers.
     */
    public double[] calculateMovementPowers() {
        vision.update(); // Always get the latest vision data first



        double shooterSpeed = .8;
        double axialPower = 0;
        double lateralPower = 0;
        double yawPower = 0;





        if (vision.isTagVisible()) {
            // A tag is visible, so we can perform our calculations.
            telemetry.addData("Aiming", "Active");

            double XOffset = vision.getX() - TARGET_X_OFFSET;
            double Bearing = vision.getBearing();
            double Range = vision.getRange();

            /*
            if (XOffset > X_TOLERANCE){
                axialPower = -1;
            }
            else if (XOffset < -X_TOLERANCE){
                axialPower = 1;
            } */

            if (Range > TARGET_RANGE + TARGET_TOLERANCE){
                lateralPower = 1;
            }
            else if (Range < TARGET_RANGE - TARGET_TOLERANCE){
                lateralPower = -1;
            }

            if (Bearing > BEARING_TOLERANCE){
                yawPower = -1;
            }
            else if (Bearing < -BEARING_TOLERANCE){
                yawPower = 1;
            }

            telemetry.addData("XOFFSET = ", XOffset);
            telemetry.addData("Bearing = ", Bearing);
            telemetry.addData("Range = ", Range);


            // fill with data to modify axial power lateral, yaw and such., and return to apply to robot.

        } else {
            // No tag is visible, so all calculated powers should be zero.
            telemetry.addData("Aiming", "Inactive (No Tag)");


            // RESET VARIABLES SO IT ISNT AIMING NOWHERE, when no tag detected
            axialPower = 0;
            lateralPower = 0;
            yawPower = 0;
        }

        // Return the calculated powers in a standard order.
        return new double[]{axialPower, lateralPower, yawPower, .2, shooterSpeed};
    }
}
