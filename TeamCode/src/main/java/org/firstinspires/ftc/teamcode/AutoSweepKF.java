package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name="AutoSweep kF Tuner", group="Tuning")
public class AutoSweepKF extends LinearOpMode {

    public static String motorName = "motor";       // motor config name
    public static double ticksPerRev = 28;       // your motor’s encoder ticks per rev

    // Feedforward only (no PID)
    public static double kF = 0.0002;               // tune this!
    public static double nominalVoltage = 12.0;

    // Sweep configuration
    public static double[] rpmTargets = {1000, 2500, 4000, 5500};  // test points
    public static double holdTime = 3.0;            // seconds at each target
    public static boolean enableVoltageComp = true;

    private DcMotorEx motor;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Ready to run sweep. Open Dashboard for data.");
        telemetry.update();

        waitForStart();

        for (double targetRpm : rpmTargets) {
            if (!opModeIsActive()) break;

            long startTime = System.currentTimeMillis();

            while (opModeIsActive() &&
                    (System.currentTimeMillis() - startTime) < holdTime * 1000) {

                double targetTps = rpmToTps(targetRpm);

                // measure actual velocity
                double measuredTps = motor.getVelocity();
                double measuredRpm = tpsToRpm(measuredTps);

                // feedforward with optional voltage comp
                double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
                double voltScale = (enableVoltageComp && batteryVoltage > 1e-3)
                        ? (nominalVoltage / batteryVoltage)
                        : 1.0;
                double ff = kF * targetTps * voltScale;

                double power = clamp(ff, -1.0, 1.0);
                motor.setPower(power);

                // log to dashboard
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("target_rpm", targetRpm);
                packet.put("measured_rpm", measuredRpm);
                packet.put("error_rpm", targetRpm - measuredRpm);
                packet.put("power", power);
                packet.put("voltage", batteryVoltage);
                packet.put("voltScale", voltScale);
                dashboard.sendTelemetryPacket(packet);

                telemetry.addData("Target RPM", targetRpm);
                telemetry.addData("Measured RPM", measuredRpm);
                telemetry.addData("Error", targetRpm - measuredRpm);
                telemetry.addData("Power", power);
                telemetry.update();
            }

            motor.setPower(0);
            sleep(1000); // pause between points
        }

        motor.setPower(0);
    }

    private double rpmToTps(double rpm) {
        return (rpm * ticksPerRev) / 60.0;
    }

    private double tpsToRpm(double tps) {
        return (tps * 60.0) / ticksPerRev;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}