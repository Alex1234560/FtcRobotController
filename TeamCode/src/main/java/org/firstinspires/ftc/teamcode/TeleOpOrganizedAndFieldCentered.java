package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOpOrganizedAndFieldCentered") // Gave it a more descriptive name for the DS
public class TeleOpOrganizedAndFieldCentered extends LinearOpMode {

    // --- Robot Hardware & Core Systems ---
    private Drive robotDrive;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    // --- Subsystem Hardware: Intake & Shooter ---
    private DcMotorEx IntakeMotor;
    private DcMotor StopIntakeMotor;
    private Servo ShooterServo;
    private DcMotorEx ShooterMotor;

    // --- Control & State Variables ---
    private double shooterMotorSpeed = 0.8;
    private boolean shooterMotorOn = false;
    private boolean wasXButtonPressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    // You could also add your vision and aiming controllers here if you re-enable them
    // private AprilTagVision vision;
    // private AimingController aimingController;

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        initializeHardware();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData(">", "Press START to begin.");
        telemetry.update();

        waitForStart();
        runtime.reset();
        imu.resetYaw(); // Zero the IMU heading at the start of the match

        // --- MAIN LOOP ---
        while (opModeIsActive()) {
            // Handle all robot actions in separate methods for clarity
            handleDriving();
            handleIntake();
            handleShooter();

            // Update all telemetry at the end of the loop
            updateTelemetry();
        }
    }

    /**
     * Initializes all hardware components and sets their initial states.
     */
    private void initializeHardware() {
        // Drivetrain & IMU
        robotDrive = new Drive(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure the IMU. This is critical for field-centric drive.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Intake & Shooter hardware
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterServo = hardwareMap.get(Servo.class, "ShooterServo");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");

        // Set motor modes
        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial servo position
        ShooterServo.setPosition(RobotConstants.SERVO_IDLE_POSITION);
    }

    /**
     * Handles all drivetrain logic, including field-centric calculations.
     */
    private void handleDriving() {
        // Get the robot's heading in radians for the trig functions
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Get joystick inputs
        double axial = -gamepad1.left_stick_y;   // Forward is positive
        double lateral = gamepad1.left_stick_x; // Right is positive
        double yaw = gamepad1.right_stick_x;      // Clockwise is positive

        // Calculate field-centric vectors
        double fieldCentricAxial = axial * Math.cos(robotHeading) - lateral * Math.sin(robotHeading);
        double fieldCentricLateral = axial * Math.sin(robotHeading) + lateral * Math.cos(robotHeading);

        // Calculate dynamic speed based on the right trigger
        double speed = 0.5 + (gamepad1.right_trigger / 2.0);

        // Command the robot to move using the field-centric values
        robotDrive.move(fieldCentricAxial, fieldCentricLateral, yaw, speed);
    }

    /**
     * Handles the logic for the intake and stop-intake motors based on gamepad 2 inputs.
     */
    private void handleIntake() {
        // Determine which trigger has priority for intake power
        double controllingTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        double intakeVelocity = IntakeMotor.getVelocity(); // Ticks per second



        double intake=0;
        if ( intakeVelocity >= 0 && gamepad2.left_bumper){
            intake = controllingTrigger;
        }
        else if (intakeVelocity <= 0){
            intake = -controllingTrigger;
        }

        double stopIntakePower = -gamepad2.right_trigger;

        IntakeMotor.setPower(intake);
        StopIntakeMotor.setPower(stopIntakePower);
    }

    /**
     * Handles all logic for the shooter motor and servo.
     */
    private void handleShooter() {
        // --- Shooter Motor Speed Control (D-pad) ---
        boolean isDpadUpPressed = gamepad2.dpad_up;
        boolean isDpadDownPressed = gamepad2.dpad_down;

        if (isDpadDownPressed && !wasDpadDownPressed && shooterMotorSpeed > 0) {
            shooterMotorSpeed -= 0.05;
        } else if (isDpadUpPressed && !wasDpadUpPressed && shooterMotorSpeed < 1) {
            shooterMotorSpeed += 0.05;
        }
        wasDpadUpPressed = isDpadUpPressed;
        wasDpadDownPressed = isDpadDownPressed;

        // --- Shooter Motor Toggle (X Button) ---
        boolean isXPressed = gamepad2.x;
        if (isXPressed && !wasXButtonPressed) {
            shooterMotorOn = !shooterMotorOn;
        }
        wasXButtonPressed = isXPressed;

        // --- Set Motor Power ---
        /*if (gamepad2.b) {
            ShooterMotor.setPower(-0.40); // Cycling Balls Mode
        } else */
        if (shooterMotorOn) {
            ShooterMotor.setPower(-shooterMotorSpeed);
        } else {
            ShooterMotor.setPower(0);
        }

        // --- Shooter Servo Control (Y Button) ---
        boolean isNormalShoot = gamepad2.y && (Math.abs(ShooterMotor.getVelocity()) > 1100);
        boolean isOverrideShoot = gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.y;

        if (isNormalShoot || isOverrideShoot) {
            ShooterServo.setPosition(RobotConstants.SERVO_SHOOT_POSITION);
        } else {
            ShooterServo.setPosition(RobotConstants.SERVO_IDLE_POSITION);
        }
    }

    /**
     * Updates and displays all relevant data to the Driver Station.
     */
    private void updateTelemetry() {
        telemetry.addData("--- Drive ---", "");
        telemetry.addData("Heading (Z)", "%.2f deg", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));

        telemetry.addData("--- Shooter ---", "");
        telemetry.addData("Power Setting", "%.2f", shooterMotorSpeed);
        telemetry.addData("Velocity (TPS)", "%.1f", ShooterMotor.getVelocity());
        telemetry.addData("Status", shooterMotorOn ? "ON" : "OFF");

        //telemetry.addData("--- Intake ---", "");
        //telemetry.addData("Velocity (TPS)", "%.1f", IntakeMotor.getVelocity());

        telemetry.update();
    }
}
