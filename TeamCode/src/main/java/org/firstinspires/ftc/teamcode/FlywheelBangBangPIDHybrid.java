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

@Config
@TeleOp(name = "Flywheel (BangBang + PID)", group = "Debug")
public class FlywheelBangBangPIDHybrid extends OpMode {

    // =========================
    // Hardware names
    // =========================
    public static String FLYWHEEL_NAME = "shooter"; // Motor with built-in encoder
    public static String STUDICA_NAME  = "m4";         // Studica on motor port 4 as sensor only

    // =========================
    // Encoder scales
    // =========================
    public static boolean USE_MANUAL_MOTOR_TPR = true;
    public static double  MANUAL_MOTOR_TPR     = 28.0;  // from your one-rev test
    private double MOTOR_TICKS_PER_REV = 1.0;

    public static double STUDICA_TICKS_PER_REV = 2048.0; // from your one-rev test
    public static boolean USE_STUDICA_FOR_CONTROL = true;

    // =========================
    // Targets (RPM)
    // =========================
    public static double RPM_HIGH = 4200;
    public static double RPM_MID  = 500;
    public static double RPM_STOP = 0;

    // Live target selection
    public static boolean USE_FIXED_TARGET = true;
    public static double  FIXED_TARGET_RPM = 4100;

    // =========================
    // Hybrid control parameters
    // =========================
    // Bang-Bang region: if |error| >= BB_BAND_RPM use bang-bang behavior
    public static double BB_BAND_RPM     = 250.0;

    // Hysteresis around zero crossing to prevent rapid toggling
    public static double BB_HYST_RPM     = 75.0;

    // Bang-Bang power levels
    public static double BB_HIGH_POWER   = 1.0;   // applied when below target by more than BB_BAND_RPM
    public static double BB_LOW_POWER    = 0.0;   // applied when above target by more than BB_HYST_RPM

    // PID region: if |error| < BB_BAND_RPM, use PID trim around a nominal feedforward power
    // Simple linear feedforward: power_ff ≈ KS + KV * targetRPM
    public static double KS              = 0.05;
    public static double KV_RPM          = 0.00017;   // about 1 / 5900..4200 depending on your motor
    public static double KP_RPM          = 0.00010;
    public static double KI_RPM          = 0.001; //0.0;
    public static double KD_RPM          = 0; //0.00030;

    // Derivative on measurement reduces noise amplification
    public static boolean DERIV_ON_MEAS  = true;

    // Integral anti-windup
    public static double I_MAX_ABS       = 0.15;       // clamp on integral contribution in power units

    // Power limits and shaping
    public static double POWER_MIN       = 0.0;
    public static double POWER_MAX       = 1.0;
    public static double POWER_SLEW_PER_CYCLE = 0.02;

    // Velocity filtering for measured RPM
    public static double VEL_ALPHA       = 0.45;
    public static int    VEL_WINDOW_MS   = 5;

    // Overspeed clamp: if measured > target + this margin, cut power to LOW for one cycle
    public static double OVER_RPM_CLAMP  = 200.0;

    // =========================
    // Telemetry controls
    // =========================
    public static boolean SEND_DASH = true;
    public static boolean VERBOSE_RC = true;
    public static int     RC_EVERY_N_LOOPS = 1;

    // =========================
    // Current reporting
    // =========================
    public static boolean REPORT_CURRENT = true;
    public static double  CURRENT_ALPHA  = 0.30;   // 0 no smoothing, 1 heavy smoothing
    private double motorCurrentA = 0.0;
    private double motorCurrentASmoothed = 0.0;

    // =========================
    // Members
    // =========================
    private DcMotorEx flywheel;
    private DcMotorEx studicaEnc;

    private double lastPower = 0.0;
    private int    loopCount = 0;

    // Signs so positive power => positive RPM
    private int motorSign = +1;
    private int studicaSign = -1;

    // Windowed velocity state
    private int  motorPosHist = 0;
    private long motorTimeHistNs = 0;
    private int  studicaPosHist = 0;
    private long studicaTimeHistNs = 0;

    // PID state
    private double errPrev = 0.0;
    private double errInt  = 0.0;
    private double measPrev = 0.0;

    @Override
    public void init() {
        flywheel   = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        studicaEnc = hardwareMap.get(DcMotorEx.class, STUDICA_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        studicaEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        studicaEnc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        studicaEnc.setPower(0.0);
        studicaEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        refreshMotorTPR(true);

        // Initialize PID state
        errPrev = 0.0;
        errInt  = 0.0;
        measPrev = 0.0;
        lastPower = 0.0;
    }

    @Override
    public void loop() {
        // Target
        double targetRpm = USE_FIXED_TARGET ? FIXED_TARGET_RPM : RPM_MID;

        // Velocities
        double motorVelTps   = velTpsWindowed(flywheel, motorSign, VEL_WINDOW_MS);
        double studicaVelTps = velTpsWindowed(studicaEnc, studicaSign, VEL_WINDOW_MS);
        double motorRpm   = tpsToRpm(motorVelTps, MOTOR_TICKS_PER_REV);
        double studicaRpm = tpsToRpm(studicaVelTps, STUDICA_TICKS_PER_REV);
        double measuredRpm = USE_STUDICA_FOR_CONTROL ? studicaRpm : motorRpm;

        // Measured RPM low-pass for PID derivative if desired
        double measFilt = VEL_ALPHA * measuredRpm + (1.0 - VEL_ALPHA) * measPrev;

        // Error
        double err = targetRpm - measuredRpm;

        // Hybrid mode selection
        boolean inBangBang = Math.abs(err) >= BB_BAND_RPM;

        double powerCmd;
        String mode;

        if (inBangBang) {
            // Hysteresis: if below target by more than BB_BAND_RPM, drive HIGH
            // if above target by more than BB_HYST_RPM, drive LOW
            if (measuredRpm <= targetRpm - BB_BAND_RPM) {
                powerCmd = BB_HIGH_POWER;
                mode = "BB_HIGH";
            } else if (measuredRpm >= targetRpm + BB_HYST_RPM) {
                powerCmd = BB_LOW_POWER;
                mode = "BB_LOW";
            } else {
                // inside hysteresis, hold last command
                powerCmd = lastPower;
                mode = "BB_HOLD";
            }
        } else {
            // PID region around target with simple feedforward
            double ff = KS + KV_RPM * targetRpm;

            // Derivative term
            double deriv;
            if (DERIV_ON_MEAS) {
                deriv = -(measFilt - measPrev); // derivative on measurement
            } else {
                deriv = (err - errPrev);
            }

            // Integral with clamp
            errInt += err;
            double iTerm = KI_RPM * errInt;
            if (iTerm > I_MAX_ABS) {
                iTerm = I_MAX_ABS;
                // optional back-calculation
            } else if (iTerm < -I_MAX_ABS) {
                iTerm = -I_MAX_ABS;
            }

            double u = ff
                    + KP_RPM * err
                    + iTerm
                    + KD_RPM * deriv;

            powerCmd = u;
            mode = "PID";
        }

        // Overspeed clamp
        if (measuredRpm > targetRpm + OVER_RPM_CLAMP) {
            powerCmd = Math.min(powerCmd, BB_LOW_POWER);
            mode = mode + "+CLAMP";
        }

        // Limits and slew
        powerCmd = clamp(powerCmd, POWER_MIN, POWER_MAX);
        if (POWER_SLEW_PER_CYCLE > 0) {
            powerCmd = slew(powerCmd, lastPower, POWER_SLEW_PER_CYCLE);
        }

        // Apply
        flywheel.setPower(powerCmd);

        // Update states
        lastPower = powerCmd;
        errPrev = err;
        measPrev = measFilt;

        // Current draw
        if (REPORT_CURRENT) {
            double iA = flywheel.getCurrent(CurrentUnit.AMPS);
            motorCurrentA = iA;
            motorCurrentASmoothed = CURRENT_ALPHA * iA + (1.0 - CURRENT_ALPHA) * motorCurrentASmoothed;
        }

        // Telemetry
        if ((loopCount++ % Math.max(1, RC_EVERY_N_LOOPS)) == 0) {
            telemetry.addLine("Flywheel Hybrid Control");
            telemetry.addData("Mode", mode);
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Measured RPM", measuredRpm);
            telemetry.addData("Motor RPM", motorRpm);
            telemetry.addData("Studica RPM", studicaRpm);
            telemetry.addData("Error RPM", err);
            telemetry.addData("Power", powerCmd);
            telemetry.addData("TPR motor/studica", "%.1f / %.1f", MOTOR_TICKS_PER_REV, STUDICA_TICKS_PER_REV);
            telemetry.addData("Sensor", USE_STUDICA_FOR_CONTROL ? "Studica" : "Motor");
            if (REPORT_CURRENT) {
                telemetry.addData("Current A (inst)", motorCurrentA);
                telemetry.addData("Current A (avg)",  motorCurrentASmoothed);
            }
            if (VERBOSE_RC) {
                telemetry.addData("BB band / hyst", "%.0f / %.0f rpm", BB_BAND_RPM, BB_HYST_RPM);
                telemetry.addData("FF KS/KV", "%.3f / %.6f", KS, KV_RPM);
                telemetry.addData("PID Kp/Ki/Kd", "%.6f / %.6f / %.6f", KP_RPM, KI_RPM, KD_RPM);
            }
            telemetry.update();
        }

        if (SEND_DASH) {
            TelemetryPacket p = new TelemetryPacket();
            p.put("mode", inBangBang ? 1 : 0); // 1 = Bang-Bang, 0 = PID
            p.put("target_rpm", targetRpm);
            p.put("measured_rpm", measuredRpm);
            p.put("error_rpm", err);
            p.put("power", powerCmd);
            p.put("motor_rpm", motorRpm);
            p.put("studica_rpm", studicaRpm);
            p.put("bb_band_rpm", BB_BAND_RPM);
            p.put("bb_hyst_rpm", BB_HYST_RPM);
            if (REPORT_CURRENT) {
                p.put("motor_current_a_inst", motorCurrentA);
                p.put("motor_current_a_avg",  motorCurrentASmoothed);
            }
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    // =========================
    // Helpers
    // =========================
    private void refreshMotorTPR(boolean force) {
        if (force || true) {
            MOTOR_TICKS_PER_REV = flywheel.getMotorType().getTicksPerRev();
            if (USE_MANUAL_MOTOR_TPR) MOTOR_TICKS_PER_REV = MANUAL_MOTOR_TPR;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double slew(double target, double prev, double step) {
        if (target > prev + step) return prev + step;
        if (target < prev - step) return prev - step;
        return target;
    }

    private static double tpsToRpm(double ticksPerSec, double ticksPerRev) {
        return ticksPerSec * 60.0 / ticksPerRev;
    }

    // Windowed velocity using position deltas, with Hub velocity fallback for short windows
    private double velTpsWindowed(DcMotorEx m, int sign, int windowMs) {
        long now = System.nanoTime();
        int pos = sign * m.getCurrentPosition();
        if (m == flywheel) {
            if (motorTimeHistNs == 0) {
                motorTimeHistNs = now;
                motorPosHist = pos;
                return sign * m.getVelocity();
            }
            long dtNs = now - motorTimeHistNs;
            if (dtNs < windowMs * 1_000_000L) return sign * m.getVelocity();
            double dpos = pos - motorPosHist;
            double dt = dtNs / 1e9;
            motorTimeHistNs = now;
            motorPosHist = pos;
            return dpos / dt;
        } else {
            if (studicaTimeHistNs == 0) {
                studicaTimeHistNs = now;
                studicaPosHist = pos;
                return sign * m.getVelocity();
            }
            long dtNs = now - studicaTimeHistNs;
            if (dtNs < windowMs * 1_000_000L) return sign * m.getVelocity();
            double dpos = pos - studicaPosHist;
            double dt = dtNs / 1e9;
            studicaTimeHistNs = now;
            studicaPosHist = pos;
            return dpos / dt;
        }
    }
}
