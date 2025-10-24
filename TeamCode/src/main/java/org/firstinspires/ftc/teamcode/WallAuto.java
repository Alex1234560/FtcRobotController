package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AprilTagVision; // <-- IMPORT YOUR NEW CLASS


import com.qualcomm.robotcore.eventloop.opmode.Disabled; // Remove this too
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoController;

//@Disabled //Remove If You wAnt this to show up on the Driver Station


@Autonomous
public class WallAuto extends OpMode {
    private Drive robotDrive;
    private AprilTagVision vision;

    private DcMotor IntakeMotor = null;
    private DcMotor StopIntakeMotor = null;
    private Servo ShooterServo;
    private DcMotorEx ShooterMotor = null;

    private ElapsedTime shooterTimer = new ElapsedTime();
    private ElapsedTime autoTimer = new ElapsedTime();    // NEW: A second timer for the overall auto routin
    private boolean isShooting = false;

    private int BallsToShot=3;
    private int BallsShot=0;

    @Override
    public void init() {


        robotDrive = new Drive(hardwareMap);

        IntakeMotor = hardwareMap.get(DcMotor.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");

        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vision = new AprilTagVision(hardwareMap, "Webcam 1");

        shooterTimer.reset();
        autoTimer.reset(); // Reset the new timer as well

    }

    @Override
    public void loop() {
        vision.update();
        double autoTime = autoTimer.seconds(); // Get the time from the new timer
        double shooterVelocity = Math.abs(ShooterMotor.getVelocity()); // Ticks per second

        double yaw = 0;
        double speed =0.15;
        double axial = 0;
        double lateral = 0;


        ShooterMotor.setPower(0);
        if (shooterVelocity < 1500 && BallsToShot!=BallsShot) {
            ShooterMotor.setPower(-1);
        }
        else if(BallsToShot!=BallsShot) { ShooterMotor.setPower(-.75); }


        if (vision.isTagVisible()) {

            // A tag is visible! Get the data from our helper methods.
            double range = vision.getRange();
            double bearing = vision.getBearing();
            double tag_yaw = vision.getYaw();
            int tagID = vision.getID();
            double xOffset = vision.getX(); // Get the new X value
            double yOffset = vision.getY(); // Get the new Y value

            /*telemetry.addData("Status", "Tag Detected!");
            telemetry.addData("Tag ID", tagID);
            telemetry.addData("Range", "%.1f in", range);
            telemetry.addData("Bearing", "%.1f deg", bearing);
            telemetry.addData("Yaw", "%.1f deg", tag_yaw);
            telemetry.addData("X Offset", "%.1f in", xOffset);
            telemetry.addData("Y Offset", "%.1f in", yOffset);*/

            lateral = 0;

            /*if (xOffset > 5){
                axial = -1;
            }
            else if (xOffset < -5){
                axial = +1;
            }
            else{axial=0;
            }*/


        } else {
            // --- TAG IS NOT VISIBLE ---
            // Display a different status message. This will overwrite the old data.
            telemetry.addData("Tag = ", "N/A");
            if (autoTime < 3){
            lateral = -1;
            }
        }
        telemetry.addData("ShooterSpeed= ", shooterVelocity);
        robotDrive.move(axial, lateral, yaw, speed);

        // Check if we should START shooting (and are not already in the middle of a shot)
        if (axial == 0 && lateral == 0 && yaw == 0 && shooterVelocity > 1700 && shooterVelocity < 2000 && !isShooting && BallsShot <= BallsToShot) {
            isShooting = true;          // Set our state to "shooting"

            shooterTimer.reset();       // Start the timer
            ShooterServo.setPosition(RobotConstants.SERVO_SHOOT_POSITION); // Move the servo to shoot
            BallsShot ++;
        }

        // If we are in the "shooting" state, check if one second has passed
        if (isShooting && shooterTimer.seconds() >= 1.5) {


            ShooterServo.setPosition(RobotConstants.SERVO_IDLE_POSITION); // Return servo to rest position
            IntakeMotor.setPower(-.5);
            StopIntakeMotor.setPower(-.6);

            if (shooterTimer.seconds()>=4.5){
                isShooting = false;
                IntakeMotor.setPower(0);
                StopIntakeMotor.setPower(0);
            }  // If so, stop shooting and reset our state

        } else if (!isShooting) {
            ShooterServo.setPosition(RobotConstants.SERVO_IDLE_POSITION); // Ensure servo is at rest if not shooting
        }





        telemetry.update(); // This was missing, but is crucial for OpMode



    }
}



