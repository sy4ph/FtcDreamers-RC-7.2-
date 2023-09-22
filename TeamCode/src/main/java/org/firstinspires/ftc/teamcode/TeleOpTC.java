package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name = "Standard Configuration OpMode", group = "Linear Opmode")
public class TeleOpTC extends OpMode {
    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    public static double MULTIPLIER = 1.0;
    public OpenCvCamera webcam;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    BNO055IMU imu;
    Orientation Angles = new Orientation();

    private final ElapsedTime runtime = new ElapsedTime();
    // Init standard configuration from StandardConfig class
    StandartConfig robot = new StandartConfig();
    private boolean changesMade = false;

    public void init() {
        robot.initTele(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    public void start() {
        runtime.reset();
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        robot.motorHand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorHand.setPower(-0.3);
//        try {
//            sleep(1500);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        robot.motorHand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorHand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {

        double max;
/**
 * I have no idea how this stuff works. It just does.
 */
//todo 28/02/22 please rewrite this
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double main = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double leftFrontPower = strafe+yaw;
        double rightBackPower = yaw-strafe;
        double rightFrontPower = main+yaw;
        double leftBackPower = yaw-main;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        robot.motorFrontLeft.setPower(leftFrontPower*MULTIPLIER);
        robot.motorFrontRight.setPower(rightFrontPower*MULTIPLIER);
        robot.motorBackLeft.setPower(leftBackPower*MULTIPLIER);
        robot.motorBackRight.setPower(rightBackPower*MULTIPLIER);
        /**
         * Here should be code for hand *todo
         */
//        robot.motorFrontLeft.setPower(leftFrontPower*0.5);
//        robot.motorFrontRight.setPower(rightFrontPower*0.5);
//        robot.motorBackLeft.setPower(leftBackPower*0.5);
//        robot.motorBackRight.setPower(rightBackPower*0.5);

        // Telemetry of hand DCMotor and runtime.
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Power (LeftFront)",leftFrontPower);
        telemetry.addData("Power (LeftBack)",leftBackPower);
        telemetry.addData("Power (RightBack)",rightBackPower);
        telemetry.addData("Power (RightFront)",rightFrontPower);
        telemetry.update();

    }

    public void stop() {

    }
}