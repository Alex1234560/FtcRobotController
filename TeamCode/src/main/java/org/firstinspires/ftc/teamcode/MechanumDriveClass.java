package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * MecanumDrive: simple reusable helper for FTC mecanum drivetrains.
 *
 * Usage:
 *   drive = new MechanumDrive(hardwareMap,
 *       new MechanumDrive.Config()
 *           .leftFrontName("left_front_drive").leftFrontDir(DcMotorSimple.Direction.REVERSE)
 *           .leftBackName("left").leftBackDir(DcMotorSimple.Direction.REVERSE)
 *           .rightFrontName("right_front_drive").rightFrontDir(DcMotorSimple.Direction.FORWARD)
 *           .rightBackName("right").rightBackDir(DcMotorSimple.Direction.FORWARD)
 *           .deadband(0.03)   // optional
 *           .maxPower(1.0)    // optional
 *   );
 *
 *   // in your loop:
 *   drive.drive(axial, lateral, yaw);
 */
public class MechanumDriveClass extends OpMode {

    // ---------- Public configuration container ----------
    public static class Config {
        private String leftFrontName    = "left_front_drive";
        private String leftBackName     = "left_back_drive";
        private String rightFrontName   = "right_front_drive";
        private String rightBackName    = "right_back_drive";

        private DcMotorSimple.Direction leftFrontDir  = DcMotorSimple.Direction.REVERSE;
        private DcMotorSimple.Direction leftBackDir   = DcMotorSimple.Direction.REVERSE;
        private DcMotorSimple.Direction rightFrontDir = DcMotorSimple.Direction.FORWARD;
        private DcMotorSimple.Direction rightBackDir  = DcMotorSimple.Direction.FORWARD;

        private boolean useEncoders = false;
        private double deadband = 0.05;
        private double maxPower = 1.0;

        public Config leftFrontName(String v) { this.leftFrontName = v; return this; }
        public Config leftBackName(String v) { this.leftBackName = v; return this; }
        public Config rightFrontName(String v) { this.rightFrontName = v; return this; }
        public Config rightBackName(String v) { this.rightBackName = v; return this; }

        public Config leftFrontDir(DcMotorSimple.Direction d) { this.leftFrontDir = d; return this; }
        public Config leftBackDir(DcMotorSimple.Direction d) { this.leftBackDir = d; return this; }
        public Config rightFrontDir(DcMotorSimple.Direction d) { this.rightFrontDir = d; return this; }
        public Config rightBackDir(DcMotorSimple.Direction d) { this.rightBackDir = d; return this; }

        public Config useEncoders(boolean v) { this.useEncoders = v; return this; }
        public Config deadband(double v) { this.deadband = v; return this; }
        public Config maxPower(double v) { this.maxPower = v; return this; }
    }

    // ---------- Private members ----------
    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    private double deadband;
    private double maxPower;

    // ---------- Construction ----------
    public MechanumDrive(HardwareMap hw, Config cfg) {
        leftFront  = hw.get(DcMotor.class, cfg.leftFrontName);
        leftBack   = hw.get(DcMotor.class, cfg.leftBackName);
        rightFront = hw.get(DcMotor.class, cfg.rightFrontName);
        rightBack  = hw.get(DcMotor.class, cfg.rightBackName);

        // Directions
        leftFront.setDirection(cfg.leftFrontDir);
        leftBack.setDirection(cfg.leftBackDir);
        rightFront.setDirection(cfg.rightFrontDir);
        rightBack.setDirection(cfg.rightBackDir);

        // Run mode
        if (cfg.useEncoders) {
            setAllRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            setAllRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.deadband = cfg.deadband;
        this.maxPower = clipAbs(cfg.maxPower, 0, 1);
    }

    // ---------- Main API ----------
    /** Feed joystick values: axial=fwd/back, lateral=strafe, yaw=rotate */
    public void drive(double axial, double lateral, double yaw) {
        axial   = applyDeadband(axial, deadband);
        lateral = applyDeadband(lateral, deadband);
        yaw     = applyDeadband(yaw, deadband);

        // Mecanum math
        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        // Normalize
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }

        // Cap power
        lf *= maxPower; rf *= maxPower; lb *= maxPower; rb *= maxPower;

        // Apply to motors
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void setDirections(DcMotorSimple.Direction lf,
                              DcMotorSimple.Direction lb,
                              DcMotorSimple.Direction rf,
                              DcMotorSimple.Direction rb) {
        leftFront.setDirection(lf);
        leftBack.setDirection(lb);
        rightFront.setDirection(rf);
        rightBack.setDirection(rb);
    }

    public void setDeadband(double v) { this.deadband = Math.max(0, v); }
    public void setMaxPower(double v) { this.maxPower = clipAbs(v, 0, 1); }

    // ---------- Internals ----------
    private void setAllRunMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    private void setAllZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        leftFront.setZeroPowerBehavior(zpb);
        rightFront.setZeroPowerBehavior(zpb);
        leftBack.setZeroPowerBehavior(zpb);
        rightBack.setZeroPowerBehavior(zpb);
    }

    private static double applyDeadband(double v, double db) {
        return (Math.abs(v) < db) ? 0.0 : v;
    }

    private static double clipAbs(double v, double lo, double hi) {
        double a = Math.abs(v);
        return Math.copySign(Math.max(lo, Math.min(hi, a)), v);
    }
}
