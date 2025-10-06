package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Config
@TeleOp(name = "Flywheel (Live FF/PID tuning)", group = "Debug")
public class FlywheelNextControlHubVelocity extends OpMode {

    // =========================
    // Hardware names
    // =========================
    public static String FLYWHEEL_NAME = "left_drive"; // Motor with built-in encoder
    public static String STUDICA_NAME  = "m4";         // Studica on motor port 4 as sensor only

    // =========================
    // Encoder scales
    // =========================
    public static boolean USE_MANUAL_MOTOR_TPR = true;
    public static double  MANUAL_MOTOR_TPR     = 28.0;   // from your one-rev test
    private double MOTOR_TICKS_PER_REV = 1.0;

    public static double STUDICA_TICKS_PER_REV = 2048.0; // from your one-rev test

    public static boolean USE_STUDICA_FOR_CONTROL = true; // set true if you want Studica as control source

    // =========================
    // Targets (RPM)
    // =========================
    public static double RPM_HIGH = 4200;
    public static double RPM_MID  = 500;
    public static double RPM_STOP = 0;

    // =========================
    // Controller (RPM domain)  Power ≈ KS + KV_RPM * rpm + PID trim
    // =========================
    public static double KS      = 0.05; //0.08;
    public static double KV_RPM  = 0.00017; // 1.0 / 4200.0;
    public static double KA_RPMs = 0.0;    // normally 0 for flywheel

    public static double KP_RPM  = 0.00001; //0.0012;
    public static double KI_RPM  = 0.0;
    public static double KD_RPM  = 0.00010;

    // Filter inside controller
    public static double VEL_ALPHA = 0.45;

    // Command shaping
    public static double POWER_SLEW_PER_CYCLE = 0.02;

    // Window for velocity estimate (ms)
    public static int VEL_WINDOW_MS = 5;

    // Live rebuild switch (press from Dashboard when you want to force a rebuild)
    public static boolean REBUILD_NOW = false;

    // =========================
    // Telemetry controls
    // =========================
    public static boolean SEND_DASH = true;
    public static boolean VERBOSE_RC = true;
    public static int RC_EVERY_N_LOOPS = 1;

    // =========================
    // Members
    // =========================
    private DcMotorEx flywheel;    // powered motor
    private DcMotorEx studicaEnc;  // external encoder on port 4, sensor only

    private ControlSystem control;

    private double targetRpm = 0.0;
    private int loopCount = 0;
    private double lastPower = 0.0;

    // Signs so positive power => positive RPM
    private int motorSign = +1;
    private int studicaSign = -1; // set to match your observation; flip if needed

    // Windowed velocity state
    private int  motorPosHist = 0;
    private long motorTimeHistNs = 0;
    private int  studicaPosHist = 0;
    private long studicaTimeHistNs = 0;

    // Snapshots of last-used tunables to detect changes
    private double lastKS, lastKV, lastKA, lastKP, lastKI, lastKD, lastAlpha;
    private boolean lastUseManualTPR;
    private double lastManualTPR;
    private boolean lastUseStudicaForControl;

    @Override
    public void init() {
        flywheel   = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        studicaEnc = hardwareMap.get(DcMotorEx.class, STUDICA_NAME);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        studicaEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        studicaEnc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        studicaEnc.setPower(0.0);
        studicaEnc.setDirection(DcMotorSimple.Direction.REVERSE);

        refreshMotorTPRIfNeeded(true);
        buildController();      // builds from current KS/KV/KA and KP/KI/KD/VEL_ALPHA
        snapshotTunables();     // remember what we used last
        lastUseStudicaForControl = USE_STUDICA_FOR_CONTROL;
    }

    @Override
    public void loop() {
        // Allow live changes to TPR and controller
        refreshMotorTPRIfNeeded(false);
        rebuildControllerIfNeeded();

        // Target
        // boolean a = gamepad1.a, x = gamepad1.x, b = gamepad1.b;
        // targetRpm = a ? RPM_HIGH : x ? RPM_MID : b ? RPM_STOP : RPM_MID;
        targetRpm = RPM_MID;
        control.setGoal(new KineticState(0.0, targetRpm));

        // Velocities
        double motorVelTps   = velTpsWindowed(flywheel, motorSign, VEL_WINDOW_MS);
        double studicaVelTps = velTpsWindowed(studicaEnc, studicaSign, VEL_WINDOW_MS);

        double motorRpm   = tpsToRpm(motorVelTps, MOTOR_TICKS_PER_REV);
        double studicaRpm = tpsToRpm(studicaVelTps, STUDICA_TICKS_PER_REV);

        // Choose control source live
        double measuredRpm = USE_STUDICA_FOR_CONTROL ? studicaRpm : motorRpm;

        // Control
        double u = control.calculate(new KineticState(0.0, measuredRpm));
        double cmd = clamp(u, 0.0, 1.0);
        if (POWER_SLEW_PER_CYCLE > 0) cmd = slew(cmd, lastPower, POWER_SLEW_PER_CYCLE);
        lastPower = cmd;
        flywheel.setPower(cmd);

        // Telemetry
        if ((loopCount++ % Math.max(1, RC_EVERY_N_LOOPS)) == 0) {
            telemetry.addLine("Flywheel Debug");
            telemetry.addData("Target RPM", targetRpm);
            telemetry.addData("Measured RPM", measuredRpm);
            telemetry.addData("Motor RPM", motorRpm);
            telemetry.addData("Studica RPM", studicaRpm);
            telemetry.addData("u (pre-clamp)", u);
            telemetry.addData("Power (applied)", cmd);
            telemetry.addData("Motor TPR", MOTOR_TICKS_PER_REV);
            telemetry.addData("Studica TPR", STUDICA_TICKS_PER_REV);
            telemetry.addData("Control Sensor", USE_STUDICA_FOR_CONTROL ? "Studica" : "Motor");
            if (VERBOSE_RC) {
                telemetry.addData("KS/KV/KA", "%.5f / %.7f / %.5f", KS, KV_RPM, KA_RPMs);
                telemetry.addData("KP/KI/KD", "%.6f / %.6f / %.6f", KP_RPM, KI_RPM, KD_RPM);
                telemetry.addData("VEL_ALPHA", VEL_ALPHA);
                telemetry.addData("motor_pos", motorSign * flywheel.getCurrentPosition());
                telemetry.addData("studica_pos", studicaSign * studicaEnc.getCurrentPosition());
                telemetry.addData("motor_tps", motorVelTps);
                telemetry.addData("studica_tps", studicaVelTps);
            }
            telemetry.update();
        }

        if (SEND_DASH) {
            TelemetryPacket p = new TelemetryPacket();
            p.put("target_rpm", targetRpm);
            p.put("measured_rpm", measuredRpm);
            p.put("motor_rpm", motorRpm);
            p.put("studica_rpm", studicaRpm);
            p.put("u_pre", u);
            p.put("power", lastPower);
            p.put("motor_pos", motorSign * flywheel.getCurrentPosition());
            p.put("studica_pos", studicaSign * studicaEnc.getCurrentPosition());
            p.put("motor_tps", motorVelTps);
            p.put("studica_tps", studicaVelTps);
            p.put("control_sensor", USE_STUDICA_FOR_CONTROL ? 1 : 0);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    // =========================
    // Live tuning helpers
    // =========================
    private void snapshotTunables() {
        lastKS = KS; lastKV = KV_RPM; lastKA = KA_RPMs;
        lastKP = KP_RPM; lastKI = KI_RPM; lastKD = KD_RPM;
        lastAlpha = VEL_ALPHA;
        lastUseManualTPR = USE_MANUAL_MOTOR_TPR;
        lastManualTPR = MANUAL_MOTOR_TPR;
        REBUILD_NOW = false;
    }

    private void refreshMotorTPRIfNeeded(boolean force) {
        if (force || USE_MANUAL_MOTOR_TPR != lastUseManualTPR || MANUAL_MOTOR_TPR != lastManualTPR) {
            MOTOR_TICKS_PER_REV = flywheel.getMotorType().getTicksPerRev();
            if (USE_MANUAL_MOTOR_TPR) MOTOR_TICKS_PER_REV = MANUAL_MOTOR_TPR;
            lastUseManualTPR = USE_MANUAL_MOTOR_TPR;
            lastManualTPR = MANUAL_MOTOR_TPR;
        }
    }

    private void buildController() {
        control = ControlSystem.builder()
                .velPid(KP_RPM, KI_RPM, KD_RPM)
                .basicFF(KV_RPM, KA_RPMs, KS)
                .velFilter(f -> f.lowPass(VEL_ALPHA))
                .build();
        control.setGoal(new KineticState(0.0, 0.0));
    }

    private void rebuildControllerIfNeeded() {
        boolean changed =
                REBUILD_NOW ||
                        KS != lastKS || KV_RPM != lastKV || KA_RPMs != lastKA ||
                        KP_RPM != lastKP || KI_RPM != lastKI || KD_RPM != lastKD ||
                        VEL_ALPHA != lastAlpha ||
                        USE_STUDICA_FOR_CONTROL != lastUseStudicaForControl;
        if (changed) {
            buildController();
            snapshotTunables();
            lastUseStudicaForControl = USE_STUDICA_FOR_CONTROL;
        }
    }

    // =========================
    // Math helpers
    // =========================
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

    // Windowed velocity using position deltas
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
