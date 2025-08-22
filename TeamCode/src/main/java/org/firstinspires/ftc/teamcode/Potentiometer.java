package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

    @TeleOp(name = "Concept: Potentiometer", group = "Concept")

    public class Potentiometer extends LinearOpMode {
        // Define variables for our potentiometer and motor
        AnalogInput potentiometer;
        DcMotor test_motor;

        // Define variable for the current voltage
        double currentVoltage;

        @Override
        public void runOpMode() {
            // Get the potentiometer and motor from hardwareMap
            potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
            test_motor = hardwareMap.get(DcMotor.class, "test_motor");

            // Loop while the Op Mode is running
            waitForStart();
            while (opModeIsActive()) {
                // Update currentVoltage from the potentiometer
                currentVoltage = potentiometer.getVoltage();

                // Turn the motor on or off based on the potentiometer’s position
                if (currentVoltage < 1.65) {
                    test_motor.setPower(0);
                } else {
                    test_motor.setPower(0.3);
                }

                // Show the potentiometer’s voltage in telemetry
                telemetry.addData("Potentiometer voltage", currentVoltage);
                telemetry.update();
            }
        }
    }

