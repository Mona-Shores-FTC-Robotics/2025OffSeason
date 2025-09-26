package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;

import static dev.nextftc.bindings.Bindings.button;

/**
 * Concept: Change PIDF with FTC Dashboard, with potentiometer based target velocity
 * and NextFTC button nudging for classroom demos.
 *
 * Functional parity with original:
 *  - Uses AnalogInput "potentiometer" to set target velocity
 *  - Tunes PIDF during runtime
 *  - Shows both Driver Station telemetry and Dashboard telemetry
 *  - Allows gamepad buttons to nudge P up and down
 */
@Config
@TeleOp(name = "Concept: Change PIDF with Dashboard", group = "Concept")
public class ChatGPTPIDFForDcMotorEx extends LinearOpMode {

    // Live tunables via FTC Dashboard
    public static volatile double P = 2.5;
    public static volatile double I = 0.1;
    public static volatile double D = 0.2;
    public static volatile double F = 1.0;

    // Potentiometer to velocity scale
    public static volatile double VOLTS_TO_TICKS_PER_SEC = 720.0;

    // Button nudge step for classroom demos
    public static volatile double P_STEP = 0.05;

    // Hardware
    private DcMotorEx motor;
    private AnalogInput potentiometer;

    // Dashboard
    private FtcDashboard dashboard;

    // Buttons
    private final Button gamepad1a = button(() -> gamepad1.a); // increase P
    private final Button gamepad1b = button(() -> gamepad1.b); // decrease P

    @Override
    public void runOpMode() {
        // Map hardware
        motor = hardwareMap.get(DcMotorEx.class, "left_drive");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        // Dashboard
        dashboard = FtcDashboard.getInstance();

        // Capture original PIDF for reference
        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Push initial PIDF from current config values
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F)
        );

        // Cache last values so we only write when something changed
        double lastP = Double.NaN, lastI = Double.NaN, lastD = Double.NaN, lastF = Double.NaN;

        // Configure button edge handlers
        gamepad1a.whenBecomesTrue(() -> P += P_STEP);
        gamepad1b.whenBecomesTrue(() -> P -= P_STEP);

        waitForStart();

        while (opModeIsActive()) {
            // Service button edges
            BindingManager.update();

            // Read inputs and compute target velocity
            double volts = potentiometer.getVoltage();
            double targetVel = volts * VOLTS_TO_TICKS_PER_SEC;

            // If any tunable changed on the Dashboard or via button nudges, update the Hub once
            if (P != lastP || I != lastI || D != lastD || F != lastF) {
                PIDFCoefficients coeffs = new PIDFCoefficients(P, I, D, F);
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
                lastP = P; lastI = I; lastD = D; lastF = F;
            }

            // Apply velocity control
            motor.setVelocity(targetVel);

            // Driver Station telemetry
            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (original)", "%.4f, %.4f, %.4f, %.4f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (current)", "%.4f, %.4f, %.4f, %.4f",
                    P, I, D, F);
            telemetry.addData("Target velocity", "%.1f", targetVel);
            telemetry.addData("Motor velocity", "%.1f", motor.getVelocity());
            telemetry.addData("Potentiometer voltage", "%.3f", volts);
            telemetry.update();

            // FTC Dashboard telemetry packet
            TelemetryPacket packet = new TelemetryPacket(); // new each loop to avoid stale keys
            packet.put("runtime_sec", getRuntime());
            packet.put("p_original", pidfOrig.p);
            packet.put("i_original", pidfOrig.i);
            packet.put("d_original", pidfOrig.d);
            packet.put("f_original", pidfOrig.f);
            packet.put("p_current", P);
            packet.put("i_current", I);
            packet.put("d_current", D);
            packet.put("f_current", F);
            packet.put("target_velocity", targetVel);
            packet.put("motor_velocity", motor.getVelocity());
            packet.put("potentiometer_voltage", volts);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
