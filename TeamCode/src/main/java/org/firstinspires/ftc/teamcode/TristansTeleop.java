package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Tristan's Teleop", group = "Test")
public class TristansTeleop extends LinearOpMode {

    // Settings
    double currentMode = 0;
    double lastMode = 0;
    double maxModes = 5;

    // Variables
    AnalogInput potentiometer;
    DcMotorEx motor;
    double currentVoltage;
    double motorPower;

    @Override
    public void runOpMode() {

        // Devices
        potentiometer = hardwareMap.get(AnalogInput.class,"potentiometer");
        motor = hardwareMap.get(DcMotorEx.class,"left_drive");

        waitForStart();
        while (opModeIsActive()) {
            // More variables
            currentVoltage = potentiometer.getVoltage();
            if (currentVoltage > 2.5) {currentVoltage = 2.5;}; // Incase voltage is above 2.5

            currentMode = Math.round(currentVoltage * 2);
            if (currentMode > maxModes) {currentMode = 5;}; // Incase current mode is above 5 (max modes)

            telemetry.addData("Current Mode Selected",currentMode);
            telemetry.update();

            if (lastMode != currentMode) {
                activateModes();
            }

            // Used to detect change in modes selected
            lastMode = currentMode;
        }
    }

    public void activateModes() {
        motor.setVelocity(-(currentMode/5)*2800);
    }
}
