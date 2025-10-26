/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp

public class MountDougTeleOp extends LinearOpMode {

    // Setting up drivetrain file
    private Drive robotDrive;
    private AprilTagVision vision;
    private AimingController aimingController;

    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx IntakeMotor = null;
    private DcMotor StopIntakeMotor = null;
    private Servo ShooterServo;
    private DcMotorEx ShooterMotor = null;

    double ShooterMotorSpeed = .8;






    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off
    private boolean wasXButtonPressed = false;   // Tracks the button's state from the last loop
    // ------------

    // --- D-pad tracking ---
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
// ----------------------------------------




    @Override
    public void runOpMode() {
        //Start a drive class to be able to use wheels
        robotDrive = new Drive(hardwareMap);
        vision = new AprilTagVision(hardwareMap, "Webcam 1"); // Use your webcam's configured name
        aimingController = new AimingController(vision, telemetry);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");

        // run shooter with encoder

        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double shooterVelocity = ShooterMotor.getVelocity(); // Ticks per second
            double intakeVelocity = IntakeMotor.getVelocity(); // Ticks per second


            //Start of DriveSpeedCode

            double speed = .5;
            //if (gamepad1.right_trigger ==1){speed = 1;}
            speed += gamepad1.right_trigger/2;
            // End of Drive Speed Code

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double ControllingTrigger =0;

            if (gamepad2.left_trigger > gamepad1.left_trigger){ControllingTrigger = gamepad2.left_trigger;}
            if (gamepad1.left_trigger > gamepad2.left_trigger){ControllingTrigger = gamepad1.left_trigger;}
            if (gamepad2.left_trigger == gamepad1.left_trigger){ControllingTrigger = gamepad2.left_trigger;}


            double intake=0;
            if ( intakeVelocity >= 0 && gamepad2.left_bumper){
                intake = ControllingTrigger;
            }
            else if (intakeVelocity <= 0){
                intake = -ControllingTrigger;
            }

            double StopIntake = -gamepad2.right_trigger;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x; // Note: pushing stick forward gives negative value
            double yaw = gamepad1.right_stick_x;

            robotDrive.move(axial, lateral, yaw, speed);

            //AUTO AIM CODE BELOW

            /*if (gamepad1.a||gamepad1.right_bumper) {
                // When 'A' is held, the AimingController takes over the drivetrain.

                // Call the single function to get the calculated powers
                double[] aimPowers = aimingController.calculateMovementPowers();

                // Unpack the array for clarity
                double axialPower = aimPowers[0];
                double lateralPower = aimPowers[1];
                double yawPower = aimPowers[2];
                speed = aimPowers[3];
                //ShooterMotorSpeed = aimPowers[4];

                // Command the robot to move using the calculated powers.
                // The robot will now automatically move to the target position.
                robotDrive.move(axialPower, lateralPower, yawPower, speed);

                telemetry.addData("Drive Mode", "AUTO-AIM ACTIVE");

            } else {
                // When 'A' is NOT held, use normal driver controls.


                telemetry.addData("Drive Mode", "Manual");
            } */

            //AUTOAIM CODE ENDS





            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.


            IntakeMotor.setPower(intake);
            StopIntakeMotor.setPower(StopIntake);



            //Below is motor code for the shooter motor, for gamepad


            //Binary Motor Logic For Shooter
            // --- START: New Shooter Motor Toggle Logic ---

            // 1. Get the current state of the 'x' button
            boolean isXPressed = gamepad2.x;

            // 2. Check if the button was JUST pressed (it was up, but is now down)
            if (isXPressed && !wasXButtonPressed) {
                // Flip the state: if it was on, turn it off; if it was off, turn it on.
                shooterMotorOn = !shooterMotorOn;
            }
            // 3. Update the tracking variable for the next loop
            wasXButtonPressed = isXPressed;



            // SHOOTING VARIABLE MECHANICS START
            boolean isDpadUpPressed = gamepad2.dpad_up;
            boolean isDpadDownPressed = gamepad2.dpad_down;

// Check if D-pad down was JUST pressed (rising edge detection)
            if (isDpadDownPressed && !wasDpadDownPressed && ShooterMotorSpeed > 0) {
                ShooterMotorSpeed -= 0.05;
            }
// Check if D-pad up was JUST pressed
            else if (isDpadUpPressed && !wasDpadUpPressed && ShooterMotorSpeed < 1) {
                ShooterMotorSpeed += 0.05;
            }

// VERY IMPORTANT: Update the tracking variables for the next loop cycle
            wasDpadUpPressed = isDpadUpPressed;
            wasDpadDownPressed = isDpadDownPressed;

            // SHOOTING VARIABLE MECHANICS END

            // 4. Set the motor power based on the toggle state

            if (gamepad2.b) {
                ShooterMotor.setPower(-.40); // Cycling Balls Mode
            }
            else if (shooterMotorOn) {
                ShooterMotor.setPower(-ShooterMotorSpeed); // Motor is ON
            } else {
                ShooterMotor.setPower(0);  // Motor is OFF
            }

            // --- END: New Shooter Motor Toggle Logic ---
            //Servo Motor Instructions below for gamepad
            //1100 for barely getting it out, 1000 for launching it into the robot and recycling
            if (gamepad2.y && Math.abs(shooterVelocity) > 1100) {
                // move to.40(180) degrees.
                ShooterServo.setPosition(RobotConstants.SERVO_SHOOT_POSITION);
            } else if(gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.y){
                ShooterServo.setPosition(RobotConstants.SERVO_SHOOT_POSITION);
            }
            else if (!gamepad2.y) {
                // move to .55(180) degrees.
                ShooterServo.setPosition(RobotConstants.SERVO_IDLE_POSITION);
            }
            // Servo Motor Instructions END







            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ShooterPower= ", ShooterMotorSpeed);
            telemetry.addData("ShooterMotorTickPerSecond= ", shooterVelocity);
            //telemetry.addData("Intake speed =  ", intakeVelocity);

            telemetry.update();
        }
    }
}

