package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

public class motortest extends OpMode {
        private DcMotorEx motor = null;

        public void init() {
                motor = hardwareMap.get(DcMotorEx.class, "motor");
        }
        public void start() {

        }
        public void loop() {
                motor.setPower(gamepad1.right_trigger);
        }
}
