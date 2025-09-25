package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "ImportClasses", group = "Sensor")
public class ImportClasses extends LinearOpMode {

    /* MORE variables */
    NormalizedColorSensor colorSensor;
    DcMotor motor;
    Servo servo;
    AnalogInput potentiometer;
    boolean debounce = false;
    double currentVoltage = 0;
    private Object GetColor;

    @Override
    public void runOpMode() {
        /* Variables */
        motor = hardwareMap.get(DcMotor.class, "left_drive");
        servo = hardwareMap.get(Servo.class, "left_hand");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Green: ",colors.green);
            colors = colorSensor.getNormalizedColors();
            telemetry.update();

            currentVoltage = potentiometer.getVoltage();

            if (currentVoltage > 2.5) {
                currentVoltage = 2.5;
            }

            ;      if (getClass(GetColor)) {
                motorStatus();
                motor.setPower(currentVoltage/2.5);
                servo.setPosition(currentVoltage/2.5);
                sleep(500);
                motor.setPower(-currentVoltage/2.5);
                servo.setPosition(-currentVoltage/2.5);
                sleep(500);
            }
        }
    }

    private boolean getClass(Object getColor) {
        return false;
    }

    public void motorStatus() {
        if (!debounce) {
            debounce = true;


        }
    }
}