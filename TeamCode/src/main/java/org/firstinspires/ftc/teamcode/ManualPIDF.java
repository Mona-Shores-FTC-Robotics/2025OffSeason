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

@TeleOp(name = "Manual PIDF", group = "Settings")
public class ManualPIDF extends LinearOpMode {

    // === Hardware / Units ===
    public static String motorName = "motor";     // change to match your configuration
    public static double ticksPerRev = 28;     // set correctly for YOUR motor/encoder

    // === Control Target ===
    public static boolean useRpm = true;          // if true, targetRpm is used; otherwise targetTps
    public static double targetRpm = 300.0;       // dashboard-tunable target in RPM
    public static double targetTps = 0.0;         // dashboard-tunable target in ticks/sec

    // === PIDF Gains (dashboard-tunable) ===
    public static double kP = 0.0008;
    public static double kI = 0.0000;
    public static double kD = 0.0001;
    public static double kF = 0.0000;             // base FF gain (per ticks/sec)

    // === Voltage Compensation (dashboard-tunable) ===
    public static boolean enableVoltageComp = true;
    public static double nominalVoltage = 12.0;   // 12.0 for 3S/“12V” FTC systems

    // === Safeties / Filters ===
    public static double integralMax = 1.0;       // clamp on integral accumulator (power units)
    public static double derivativeAlpha = 0.7;   // 0..1, larger = smoother derivative
    public static double powerMin = -1.0;
    public static double powerMax =  1.0;

    // === Quality-of-life ===
    public static boolean resetIntegralOnTargetChange = true;
    public static boolean brakeWhenZeroTarget = true;
    public static boolean showPerCycleTelemetry = true;

    private DcMotorEx motor;
    private final ElapsedTime timer = new ElapsedTime();

    // Sensors
    private List<VoltageSensor> voltageSensors;

    // State
    private double integral = 0.0;
    private double prevError = 0.0;
    private double filteredDeriv = 0.0;
    private double lastTargetTps = 0.0;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "left_drive");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        voltageSensors = hardwareMap.getAll(VoltageSensor.class);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        timer.reset();

        telemetry.addLine("ManualVelocityPIDF ready. Open FTC Dashboard to tune.");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double dt = Math.max(timer.seconds(), 1e-6);
            timer.reset();

            // 1) Determine target in ticks/second
            double tgtTps = useRpm ? rpmToTps(targetRpm) : targetTps;

            // Optional: reset I if target changed significantly
            if (resetIntegralOnTargetChange && Math.abs(tgtTps - lastTargetTps) > 1e-6) {
                integral = 0.0;
                prevError = 0.0;
                filteredDeriv = 0.0;
            }
            lastTargetTps = tgtTps;

            // 2) Measure current velocity (ticks/sec)
            double measuredTps = motor.getVelocity();

            // 3) PIDF math
            double error = tgtTps - measuredTps;

            // Integrator with anti-windup (accumulate in power units)
            integral += error * dt * kI;
            integral = clamp(integral, -integralMax, integralMax);

            // Derivative with simple low-pass filter on error slope
            double rawDeriv = (error - prevError) / dt;
            filteredDeriv = derivativeAlpha * filteredDeriv + (1.0 - derivativeAlpha) * rawDeriv;
            prevError = error;

            // Feedforward with voltage compensation
            double batteryVoltage = getBatteryVoltage();
            double voltScale = (enableVoltageComp && batteryVoltage > 1e-3)
                    ? (nominalVoltage / batteryVoltage)
                    : 1.0;
            double ff = kF * tgtTps * voltScale;

            // PID terms (power units)
            double pTerm = kP * error;
            double dTerm = kD * filteredDeriv;

            double power = pTerm + integral + dTerm + ff;

            // If target ~ zero, optionally hold brake and kill integrator
            if (Math.abs(tgtTps) < 1e-6 && brakeWhenZeroTarget) {
                integral = 0.0;
            }

            // 4) Apply power (clamped)
            power = clamp(power, powerMin, powerMax);
            motor.setPower(power);

            // 5) Dashboard telemetry
            if (showPerCycleTelemetry) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("dt_s", dt);
                packet.put("target_tps", tgtTps);
                packet.put("target_rpm", tpsToRpm(tgtTps));
                packet.put("measured_tps", measuredTps);
                packet.put("measured_rpm", tpsToRpm(measuredTps));
                packet.put("error_tps", error);
                packet.put("P", pTerm);
                packet.put("I", integral);
                packet.put("D", dTerm);
                packet.put("FF_base", kF * tgtTps);
                packet.put("battery_V", batteryVoltage);
                packet.put("voltScale", voltScale);
                packet.put("FF", ff);
                packet.put("power_cmd", power);
                dashboard.sendTelemetryPacket(packet);
            }

            // RC telemetry (optional)
            telemetry.addData("tgt (rpm)", tpsToRpm(tgtTps));
            telemetry.addData("meas (rpm)", tpsToRpm(measuredTps));
            telemetry.addData("Vbat", batteryVoltage);
            telemetry.addData("power", power);
            telemetry.update();
        }

        motor.setPower(0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double rpmToTps(double rpm) {
        return (rpm * ticksPerRev) / 60.0;
    }

    private double tpsToRpm(double tps) {
        return (tps * 60.0) / ticksPerRev;
    }

    /**
     * Returns a reasonable battery voltage reading.
     * We take the MAX of all VoltageSensors that report > 0 V
     * because some hubs can mirror sensors and occasionally report 0.
     */
    private double getBatteryVoltage() {
        double v = 0.0;
        for (VoltageSensor sensor : voltageSensors) {
            double reading = sensor.getVoltage();
            if (reading > v) v = reading;
        }
        return v;
    }
}
