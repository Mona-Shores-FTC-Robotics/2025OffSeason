package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Encoder TPR Test", group="Debug")
public class EncoderTPRTest extends OpMode {

    public static String MOTOR_NAME = "left_drive";
    public static String STUDICA_NAME = "m4";

    private DcMotorEx motor;
    private DcMotorEx studica;

    private int motorStart = 0;
    private int studicaStart = 0;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        studica = hardwareMap.get(DcMotorEx.class, STUDICA_NAME);

        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        studica.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // capture zero positions
        motorStart = motor.getCurrentPosition();
        studicaStart = studica.getCurrentPosition();
    }

    @Override
    public void loop() {
        int motorDelta = motor.getCurrentPosition() - motorStart;
        int studicaDelta = studica.getCurrentPosition() - studicaStart;

        telemetry.addLine("Rotate the flywheel shaft by hand EXACTLY 1 full turn");
        telemetry.addData("Motor Δticks", motorDelta);
        telemetry.addData("Studica Δticks", studicaDelta);
        telemetry.update();
    }
}
