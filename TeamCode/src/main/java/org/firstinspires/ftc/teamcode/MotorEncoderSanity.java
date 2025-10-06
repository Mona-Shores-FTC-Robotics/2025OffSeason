package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "MotorEncoderSanity (Dashboard)", group = "Debug")
public class MotorEncoderSanity extends OpMode {

    // ---------- CONFIG (tune from FTC Dashboard) ----------
    public static String MOTOR_NAME = "left_drive";
    // REV Through-Bore quadrature on motor port: 2048 CPR -> 8192 counts/rev (4x)
    public static double TICKS_PER_REV = 8192.0;

    // Direct power command (set this in Dashboard)
    public static double POWER_CMD = 0.00;          // range [-1.0, 1.0]
    public static boolean HOLD_ZERO_POWER = false;  // force power to 0 (quick stop)

    // Sign / direction tools
    public static boolean REVERSE_DIRECTION = false; // flips motor.setDirection()
    public static boolean NEGATE_VELOCITY = false;   // multiplies measured velocity by -1

    // Utilities
    public static boolean RESET_ENCODER = false;     // set true to zero encoder (edge-detected)

    // ---------- STATE ----------
    private DcMotorEx motor;
    private int lastPos = 0;
    private long lastNanos = 0;

    // Edge-detect mirrors for the dashboard booleans
    private boolean prevReverseDirection = REVERSE_DIRECTION;
    private boolean prevResetEncoder     = RESET_ENCODER;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        // We control power ourselves and read the encoder
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyDirectionFromFlag(); // set initial FORWARD/REVERSE

        lastPos = motor.getCurrentPosition();
        lastNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        // --- Handle dashboard toggles (edge-detected) ---
        if (REVERSE_DIRECTION != prevReverseDirection) {
            applyDirectionFromFlag();
            prevReverseDirection = REVERSE_DIRECTION;
        }
        if (RESET_ENCODER && !prevResetEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lastPos = motor.getCurrentPosition();
            lastNanos = System.nanoTime();
        }
        prevResetEncoder = RESET_ENCODER;

        // --- Apply commanded power ---
        double appliedPower = HOLD_ZERO_POWER ? 0.0 : clamp(POWER_CMD, -1.0, 1.0);
        motor.setPower(appliedPower);

        // --- Read sensors ---
        int pos = motor.getCurrentPosition();
        long now = System.nanoTime();

        double dt = Math.max((now - lastNanos) * 1e-9, 1e-6); // seconds
        int dpos = pos - lastPos;
        lastPos = pos;
        lastNanos = now;

        // Two velocity estimates
        double velTpsHub   = motor.getVelocity(); // ticks/sec (hub’s reading)
        double velTpsDelta = dpos / dt;           // ticks/sec (from position delta)

        if (NEGATE_VELOCITY) {
            velTpsHub   = -velTpsHub;
            velTpsDelta = -velTpsDelta;
        }

        // Convert to RPM for readability
        double rpmHub   = tpsToRpm(velTpsHub);
        double rpmDelta = tpsToRpm(velTpsDelta);

        // Sign alignment checks
        boolean alignedHub   = signSame(appliedPower, velTpsHub);
        boolean alignedDelta = signSame(appliedPower, velTpsDelta);

        // --- Driver Station telemetry ---
        telemetry.addLine("=== Motor & Encoder Sanity (Dashboard) ===");
        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.addData("Direction", motor.getDirection());
        telemetry.addData("ReverseDirection flag", REVERSE_DIRECTION);
        telemetry.addData("NegateVelocity flag", NEGATE_VELOCITY);
        telemetry.addData("HoldZeroPower", HOLD_ZERO_POWER);
        telemetry.addData("PowerCmd", "%.3f", appliedPower);

        telemetry.addLine();
        telemetry.addData("Pos (ticks)", pos);
        telemetry.addData("ΔPos (ticks)", dpos);
        telemetry.addData("Δt (s)", "%.4f", dt);

        telemetry.addLine("Velocity (ticks/s)");
        telemetry.addData("Hub  ", "%.1f", velTpsHub);
        telemetry.addData("Delta", "%.1f", velTpsDelta);

        telemetry.addLine("Velocity (RPM)");
        telemetry.addData("Hub  ", "%.1f", rpmHub);
        telemetry.addData("Delta", "%.1f", rpmDelta);

        telemetry.addLine();
        telemetry.addData("Sign Align (Hub)   +power↔+vel ?", alignedHub);
        telemetry.addData("Sign Align (Delta) +power↔+vel ?", alignedDelta);

        telemetry.addLine();
        telemetry.addData("TPR", TICKS_PER_REV);
        telemetry.update();

        // --- Optional: FTC Dashboard packet (nice graphs) ---
        TelemetryPacket p = new TelemetryPacket();
        p.put("power_cmd", appliedPower);
        p.put("pos_ticks", pos);
        p.put("vel_hub_tps", velTpsHub);
        p.put("vel_delta_tps", velTpsDelta);
        p.put("rpm_hub", rpmHub);
        p.put("rpm_delta", rpmDelta);
        p.put("aligned_hub", alignedHub ? 1 : 0);
        p.put("aligned_delta", alignedDelta ? 1 : 0);
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }

    // ---------- helpers ----------
    private void applyDirectionFromFlag() {
        motor.setDirection(REVERSE_DIRECTION
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);
    }

    private static boolean signSame(double a, double b) {
        if (Math.abs(a) < 1e-6 || Math.abs(b) < 1e-6) return true; // treat near-zero as okay
        return Math.signum(a) == Math.signum(b);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double tpsToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }
}
