package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop", group = "Linear OpMode")
public class teleop extends LinearOpMode {

    static double SLOWMO_POWER_SCALE = 0.25;

    // Slew rate limits (units per second)
    static double TRANSLATION_SLEW = 4.0;
    static double ROTATION_SLEW = 6.0;

    boolean resetReady = true;

    double lastX = 0;
    double lastY = 0;
    double lastTurn = 0;

    double headingOffset = 0;

    @Override
    public void runOpMode() {

        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------- IMU ----------
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        telemetry.addLine("Ready — press start");
        telemetry.update();

        waitForStart();
        imu.resetYaw();
        headingOffset = 0;

        double lastTime = getRuntime();

        while (opModeIsActive()) {

            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // ---------- INPUT ----------
            double rawX = gamepad1.left_stick_x;
            double rawY = -gamepad1.left_stick_y;
            double rawTurn = gamepad1.right_stick_x;

            double scale = gamepad1.left_bumper ? SLOWMO_POWER_SCALE : 1.0;

            // ---------- DRIVER-DEFINED FORWARD RESET ----------
            if (gamepad1.y && resetReady) {
                imu.resetYaw();
                headingOffset = 0;
                resetReady = false;
            }
            if (!gamepad1.y) resetReady = true;

            // ---------- IMU ----------
            double heading;
            try {
                heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + headingOffset;
            } catch (Exception e) {
                heading = 0.0; // fallback robot-centric
            }

            // ---------- FIELD-CENTRIC ----------
            double x = rawX * Math.cos(-heading) - rawY * Math.sin(-heading);
            double y = rawX * Math.sin(-heading) + rawY * Math.cos(-heading);

            // ---------- HEADING LOCK ----------
            double turn;
            if (Math.abs(rawTurn) < 0.05) {
                turn = lastTurn * 0.9; // decay toward 0 smoothly
            } else {
                turn = rawTurn;
            }

            // ---------- SLEW RATE LIMITING ----------
            x = slew(lastX, x, TRANSLATION_SLEW, dt);
            y = slew(lastY, y, TRANSLATION_SLEW, dt);
            turn = slew(lastTurn, turn, ROTATION_SLEW, dt);

            lastX = x;
            lastY = y;
            lastTurn = turn;

            // ---------- MECANUM MIX ----------
            double lf = x + y + turn;
            double lb = -x + y + turn;
            double rb = x + y - turn;
            double rf = -x + y - turn;

            // ---------- NORMALIZATION ----------
            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                            Math.max(Math.abs(lb),
                                    Math.max(Math.abs(rb), Math.abs(rf)))));

            lf /= max;
            lb /= max;
            rb /= max;
            rf /= max;

            leftFront.setPower(lf * scale);
            leftBack.setPower(lb * scale);
            rightBack.setPower(rb * scale);
            rightFront.setPower(rf * scale);

            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Slow Mode", gamepad1.left_bumper);
            telemetry.update();
        }
    }

    // ---------- Slew Helper ----------
    private double slew(double current, double target, double rate, double dt) {
        double maxDelta = rate * dt;
        double delta = target - current;
        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }
        return current + delta;
    }
}
