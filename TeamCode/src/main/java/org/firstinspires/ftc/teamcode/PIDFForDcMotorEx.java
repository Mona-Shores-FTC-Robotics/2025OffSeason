package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;

import static dev.nextftc.bindings.Bindings.*;


/**
     * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
     * This assumes that you are using a REV Robotics Control Hub or REV Robotics Expansion Hub
     * as your DC motor controller.  This OpMode uses the extended/enhanced
     * PIDF-related functions of the DcMotorEx class.
     */

    @TeleOp(name="Concept: Change PIDF", group = "Concept")
    @Config
    public class PIDFForDcMotorEx extends LinearOpMode {

        double currentVoltage;
        double targetVelocity;
        // our DC motor
        public static PIDFCoefficients pidfModified = new PIDFCoefficients();

    DcMotorEx motorExLeft;
        AnalogInput potentiometer;
        Button gamepad1a = button(() -> gamepad1.a);
        Button gamepad1b = button(() -> gamepad1.b);
        TelemetryPacket packet = new TelemetryPacket();


    public static final double NEW_P = 2.5;
        public static final double NEW_I = 0.1;
        public static final double NEW_D = 0.2;
        public static final double NEW_F = 1;
        // These values are for illustration only; they must be set
        // and adjusted for each motor based on its planned usage.

        public void runOpMode() {
            // Get reference to DC motor.
            // Since we are using the Control Hub or Expansion Hub,
            // cast this motor to a DcMotorEx object.
            motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_drive");
            potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

            // wait for start command
            waitForStart();

            // Get the PIDF coefficients for the RUN_USING_ENCODER RunMode.
            PIDFCoefficients pidfOrig = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            // Change coefficients using methods included with DcMotorEx class.
            PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            // Re-read coefficients and verify change.
             pidfModified = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            gamepad1a.whenBecomesTrue(() -> {
                pidfModified.p += 0.05;
//                motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfModified);
            });
            gamepad1b.whenBecomesTrue(() ->{
                pidfModified.p -= 0.05;
//                motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfModified);
            });

            // display info to user
            while(opModeIsActive()) {
                currentVoltage = potentiometer.getVoltage();
                targetVelocity = currentVoltage * 720;
                BindingManager.update();

                telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
                packet.put("runtime", getRuntime());

                telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                        pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
                packet.put("p(original)", pidfOrig.p);

                telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                        pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
                packet.put("p(modified)", pidfModified.p);

                telemetry.addData("Target velocity: ", targetVelocity);
                packet.put("Target velocity", targetVelocity);

                telemetry.addData("Motor velocity", motorExLeft.getVelocity());
                packet.put("Motor Velocity",motorExLeft.getVelocity());

                telemetry.addData("Potentiometer voltage", currentVoltage);
                packet.put("Potentiometer voltage",currentVoltage);

                telemetry.update();
                FtcDashboard dashboard = FtcDashboard.getInstance();
                dashboard.sendTelemetryPacket(packet);

                motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfModified);

                motorExLeft.setVelocity(targetVelocity);
            }
        }
    }
