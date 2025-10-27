package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Three independent Bang-Bang + PID hybrid flywheel controllers.
 * Each motor has fully isolated state. No shared integrators or histories.
 */
@Config
@TeleOp(name = "Flywheel (BangBang + PID) Triple", group = "Debug")
public class FlywheelBangBangPIDHybridTriple extends OpMode {

    // =========================
    // Hardware names
    // =========================
    public static String FLYWHEEL_LEFT_NAME  = "leftshooter";
    public static String FLYWHEEL_MID_NAME   = "midshooter";
    public static String FLYWHEEL_RIGHT_NAME = "rightshooter";

    // Optional: set motor directions if your build requires it
    public static DcMotorSimple.Direction LEFT_DIR  = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction MID_DIR   = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

    // =========================
    // Encoder scales
    // =========================
    public static boolean USE_MANUAL_TPR = true;
    public static double  MANUAL_TPR     = 28.0; // One-rev test value you measured

    // =========================
    // Targets (RPM) per motor
    // =========================
    public static boolean USE_FIXED_TARGETS = true;
    public static double  TARGET_L_RPM = 0;
    public static double  TARGET_M_RPM = 0;
    public static double  TARGET_R_RPM = 0;

    // Quick presets to toggle if not using fixed targets
    public static double RPM_HIGH = 4200;
    public static double RPM_MID  = 500;
    public static double RPM_STOP = 0;

    // =========================
    // Hybrid control parameters (shared)
    // =========================
    public static double BB_BAND_RPM   = 250.0;  // Bang-Bang region half-width
    public static double BB_HYST_RPM   = 75.0;   // Bang-Bang hysteresis
    public static double BB_HIGH_POWER = 1.0;
    public static double BB_LOW_POWER  = 0.0;

    // Feedforward + PID about the target when inside band
    public static double KS      = 0.05;
    public static double KV_RPM  = 0.00017;
    public static double KP_RPM  = 0.00010;
    public static double KI_RPM  = 0.001;
    public static double KD_RPM  = 0.0;

    public static boolean DERIV_ON_MEAS = true;
    public static double  I_MAX_ABS     = 0.15;

    public static double POWER_MIN = 0.0;
    public static double POWER_MAX = 1.0;
    public static double POWER_SLEW_PER_CYCLE = 0.02;

    public static double VEL_ALPHA     = 0.45; // low-pass for derivative on measurement
    public static int    VEL_WINDOW_MS = 5;    // position-delta window

    public static double OVER_RPM_CLAMP = 200.0;

    // Telemetry
    public static boolean SEND_DASH       = true;
    public static boolean VERBOSE_RC      = true;
    public static int     RC_EVERY_N_LOOPS = 1;

    public static boolean REPORT_CURRENT  = true;
    public static double  CURRENT_ALPHA   = 0.30;

    private int loopCount = 0;

    // =========================
    // Per-motor controller
    // =========================
    private static class HybridController {
        final DcMotorEx motor;
        final String    label;

        // Configuration
        double ticksPerRev;
        double lastPower = 0.0;

        // Velocity window state
        long   timeHistNs = 0;
        int    posHist    = 0;

        // PID state
        double errPrev  = 0.0;
        double errInt   = 0.0;
        double measPrev = 0.0;

        // Current smoothing
        double currentInst = 0.0;
        double currentAvg  = 0.0;

        HybridController(DcMotorEx m, String label, double tpr) {
            this.motor = m;
            this.label = label;
            this.ticksPerRev = tpr;
        }

        double update(double targetRpm) {
            double velTps = velTpsWindowed();
            double motorRpm = tpsToRpm(velTps, ticksPerRev);

            // Low-pass for derivative on measurement
            double measFilt = VEL_ALPHA * motorRpm + (1.0 - VEL_ALPHA) * measPrev;

            double err = targetRpm - motorRpm;
            boolean inBangBang = Math.abs(err) >= BB_BAND_RPM;

            double powerCmd;
            String mode;

            if (inBangBang) {
                if (motorRpm <= targetRpm - BB_BAND_RPM) {
                    powerCmd = BB_HIGH_POWER;
                    mode = "BB_HIGH";
                } else if (motorRpm >= targetRpm + BB_HYST_RPM) {
                    powerCmd = BB_LOW_POWER;
                    mode = "BB_LOW";
                } else {
                    powerCmd = lastPower; // inside hysteresis
                    mode = "BB_HOLD";
                }
            } else {
                // Feedforward + PID trim
                double ff = KS + KV_RPM * targetRpm;

                double deriv = DERIV_ON_MEAS ? -(measFilt - measPrev) : (err - errPrev);

                errInt += err;
                double iTerm = KI_RPM * errInt;
                if (iTerm > I_MAX_ABS) iTerm = I_MAX_ABS;
                if (iTerm < -I_MAX_ABS) iTerm = -I_MAX_ABS;

                double u = ff + KP_RPM * err + iTerm + KD_RPM * deriv;

                powerCmd = u;
                mode = "PID";
            }

            // Overspeed clamp
            if (motorRpm > targetRpm + OVER_RPM_CLAMP) {
                powerCmd = Math.min(powerCmd, BB_LOW_POWER);
                mode = mode + "+CLAMP";
            }

            // Limits and slew
            powerCmd = clamp(powerCmd, POWER_MIN, POWER_MAX);
            if (POWER_SLEW_PER_CYCLE > 0) {
                powerCmd = slew(powerCmd, lastPower, POWER_SLEW_PER_CYCLE);
            }

            // Apply
            motor.setPower(powerCmd);
            lastPower = powerCmd;
            errPrev = targetRpm - motorRpm;
            measPrev = measFilt;

            // Current
            try {
                currentInst = motor.getCurrent(CurrentUnit.AMPS);
            } catch (Throwable t) {
                currentInst = 0.0;
            }
            currentAvg = CURRENT_ALPHA * currentInst + (1.0 - CURRENT_ALPHA) * currentAvg;

            // Return measured RPM for external telemetry
            return motorRpm;
        }

        private double velTpsWindowed() {
            long now = System.nanoTime();
            int pos = motor.getCurrentPosition();
            if (timeHistNs == 0) {
                timeHistNs = now;
                posHist = pos;
                return motor.getVelocity(); // hub velocity on first sample
            }
            long dtNs = now - timeHistNs;
            if (dtNs < VEL_WINDOW_MS * 1_000_000L) {
                return motor.getVelocity(); // use hub velocity for short windows
            }
            double dpos = pos - posHist;
            double dt = dtNs / 1e9;
            timeHistNs = now;
            posHist = pos;
            return dpos / dt;
        }

        private static double tpsToRpm(double tps, double tpr) {
            return tps * 60.0 / tpr;
        }

        private static double clamp(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        private static double slew(double target, double prev, double step) {
            if (target > prev + step) return prev + step;
            if (target < prev - step) return prev - step;
            return target;
        }
    }

    // Controllers
    private HybridController leftCtrl;
    private HybridController midCtrl;
    private HybridController rightCtrl;

    @Override
    public void init() {
        DcMotorEx left  = hardwareMap.get(DcMotorEx.class, FLYWHEEL_LEFT_NAME);
        DcMotorEx mid   = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MID_NAME);
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, FLYWHEEL_RIGHT_NAME);

        // Modes and directions
        configureMotor(left,  LEFT_DIR);
        configureMotor(mid,   MID_DIR);
        configureMotor(right, RIGHT_DIR);

        // TPR
        double tprL = USE_MANUAL_TPR ? MANUAL_TPR : left.getMotorType().getTicksPerRev();
        double tprM = USE_MANUAL_TPR ? MANUAL_TPR : mid.getMotorType().getTicksPerRev();
        double tprR = USE_MANUAL_TPR ? MANUAL_TPR : right.getMotorType().getTicksPerRev();

        leftCtrl  = new HybridController(left,  "L", tprL);
        midCtrl   = new HybridController(mid,   "M", tprM);
        rightCtrl = new HybridController(right, "R", tprR);
    }

    private void configureMotor(DcMotorEx m, DcMotorSimple.Direction dir) {
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setDirection(dir);
        m.setPower(0.0);
    }

    @Override
    public void loop() {
        double targetL = USE_FIXED_TARGETS ? TARGET_L_RPM : RPM_MID;
        double targetM = USE_FIXED_TARGETS ? TARGET_M_RPM : RPM_MID;
        double targetR = USE_FIXED_TARGETS ? TARGET_R_RPM : RPM_MID;

        double rpmL = leftCtrl.update(targetL);
        double rpmM = midCtrl.update(targetM);
        double rpmR = rightCtrl.update(targetR);

        if ((loopCount++ % Math.max(1, RC_EVERY_N_LOOPS)) == 0) {
            telemetry.addLine("Hybrid Triple");
            telemetry.addData("Target RPM L/M/R", "%.0f / %.0f / %.0f", targetL, targetM, targetR);
            telemetry.addData("Measured RPM L/M/R", "%.0f / %.0f / %.0f", rpmL, rpmM, rpmR);
            telemetry.addData("BB band / hyst", "%.0f / %.0f rpm", BB_BAND_RPM, BB_HYST_RPM);
            if (VERBOSE_RC) {
                telemetry.addData("FF KS/KV", "%.3f / %.6f", KS, KV_RPM);
                telemetry.addData("PID Kp/Ki/Kd", "%.6f / %.6f / %.6f", KP_RPM, KI_RPM, KD_RPM);
            }
            if (REPORT_CURRENT) {
                telemetry.addData("Current L/M/R (A, avg)", "%.2f / %.2f / %.2f",
                        leftCtrl.currentAvg, midCtrl.currentAvg, rightCtrl.currentAvg);
            }
            telemetry.update();
        }

        if (SEND_DASH) {
            TelemetryPacket p = new TelemetryPacket();
            p.put("target_rpm_L", targetL);
            p.put("target_rpm_M", targetM);
            p.put("target_rpm_R", targetR);
            p.put("measured_rpm_L", rpmL);
            p.put("measured_rpm_M", rpmM);
            p.put("measured_rpm_R", rpmR);
            p.put("bb_band_rpm", BB_BAND_RPM);
            p.put("bb_hyst_rpm", BB_HYST_RPM);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}
