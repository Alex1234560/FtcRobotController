package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class encapsulates all the hardware and logic for the robot's mecanum drivetrain.
 * It makes the main OpMode cleaner by hiding the low-level details of motor control.
 */
public class Drive {

    // 1. Declare Hardware
    // These are 'private' because only the Drive class should control them directly.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    /**
     * The constructor for the Drive class.
     * This is called when you create a new "Drive" object. Its job is to set up the motors.
     * @param hardwareMap The hardware map from your OpMode, used to find the motors.
     */
    public Drive(HardwareMap hardwareMap) {
        // 2. Initialize Hardware from the hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBack");

        // 3. Set Motor Directions
        // This configuration is for a standard mecanum drive.
        // You may need to adjust this based on your robot's build.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Optional: Set brake behavior
        setBrakeMode(true);
    }

    /**
     * The main method for controlling the robot's movement.
     * It takes high-level movement commands (axial, lateral, yaw) and translates
     * them into power levels for each of the four motors.
     * @param axial   Forward/Backward power (-1.0 to 1.0)
     * @param lateral Strafe left/Right power (-1.0 to 1.0)
     * @param yaw     Turning power (-1.0 to 1.0)
     */
    public void move(double axial, double lateral, double yaw, double speed) {
        // Combine the movement requests to determine each wheel's power
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This is the same logic you had before, now inside a dedicated method.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftPower   /= max;
            rightBackPower  /= max;
        }

        // Send the calculated power to the motors
        leftFrontDrive.setPower(leftFrontPower * speed);
        rightFrontDrive.setPower(rightFrontPower * speed);
        leftBackDrive.setPower(leftPower * speed);
        rightBackDrive.setPower(rightBackPower * speed);


    }

    /**
     * A helper method to stop all drive motors.
     */
    public void stop() {
        move(0, 0, 0, 0);
    }

    /**
     * A helper method to set the brake behavior of all drive motors.
     * @param enabled If true, motors will brake when power is zero. If false, they will coast.
     */
    public void setBrakeMode(boolean enabled) {
        DcMotor.ZeroPowerBehavior behavior = enabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
