package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Config
@TeleOp(name = "Flywheel (Hub Velocity)", group = "Debug")
public class FlywheelNextControlHubVelocity extends OpMode {

    // --- Hardware / Units ---
    public static String FLYWHEEL_NAME = "left_drive";
    public static double TICKS_PER_REV = 2048; // verify for your encoder/gearbox

    // --- Targets ---
    public static double RPM_HIGH = 4200;   // gamepad1.a
    public static double RPM_MID  = 3000;   // gamepad1.x
    public static double RPM_STOP = 0;      // gamepad1.b

    // --- Velocity PID gains ---
    public static double kPv = 0.0007;
    public static double kIv = 0.0000;
    public static double kDv = 0.00015;

    // --- Feedforward (basicFF: v, a, s) ---
    public static double kV = 0.0;  // power per ticks/sec
    public static double kA = 0.0;
    public static double kS = 0.0;

    // --- Built-in LPF coefficients ---
    public static double POS_ALPHA = 0.40;  // optional for flywheel
    public static double VEL_ALPHA = 0.20;  // often enough to calm hub velocity

    // --- Debug / Output Controls ---
    public static boolean SEND_DASH = true; // dashboard graphs
    public static boolean VERBOSE_RC = true; // show full RC telemetry
    public static int RC_EVERY_N_LOOPS = 1;  // increase to 2–5 to reduce spam

    private DcMotorEx flywheel;
    private ControlSystem control;

    private double targetTps = 0.0;
    private boolean pA, pB, pX; // edge latch

    // For position-delta cross-check
    private final ElapsedTime loopTimer = new ElapsedTime();
    private int lastPos = 0;

    // Loop counter to throttle RC telemetry
    private int loopCount = 0;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // simple, hub estimates velocity
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        control = ControlSystem.builder()
                .velPid(kPv, kIv, kDv)              // feedback
               // .basicFF(kV, kA, kS)                // feedforward
                //.posFilter(f -> f.lowPass(POS_ALPHA))
                //.velFilter(f -> f.lowPass(VEL_ALPHA))
                .build();

        control.setGoal(new KineticState(0.0)); // goal is velocity (tps)
        lastPos = flywheel.getCurrentPosition();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        // --- Latch setpoint on button edge ---
        boolean a = gamepad1.a, x = gamepad1.x, b = gamepad1.b;
//        if (a && !pA) targetTps = rpmToTps(RPM_HIGH);
//        if (x && !pX) targetTps = rpmToTps(RPM_MID);
//        if (b && !pB) targetTps = rpmToTps(RPM_STOP);
//        pA = a; pX = x; pB = b;
        targetTps = rpmToTps(RPM_MID);
        control.setGoal(new KineticState(targetTps));


//        // --- Measured state (raw, before any ControlSystem filtering) ---
//        int posTicks = flywheel.getCurrentPosition();
        double velTpsHub = flywheel.getVelocity();       // hub's ticks/sec
//        double dt = clamp(loopTimer.seconds(), 1e-3, 0.05);
//        loopTimer.reset();
//
//        // Position-delta velocity for cross-check
//        int dpos = posTicks - lastPos;
//        lastPos = posTicks;
//        double velTpsPosDelta = dpos / dt;

        // --- Controller output (pre-clamp) ---
        double powerPreClamp = control.calculate(new KineticState(velTpsHub));

        // --- Clamp and apply ---
        double powerCmd = clamp(powerPreClamp, 0, 1.0); // forward-only
        flywheel.setPower(powerCmd);

        // --- RC telemetry (readable labels) ---
        if ((loopCount++ % Math.max(1, RC_EVERY_N_LOOPS)) == 0) {
            telemetry.addLine("Flywheel Debug");
            telemetry.addData("Target Speed (RPM)", tpsToRpm(targetTps));
//            telemetry.addData("Position (ticks)", posTicks);
//            telemetry.addData("ΔPosition (ticks)", dpos);
//            telemetry.addData("Loop Δt (s)", dt);

            telemetry.addData("Velocity – Hub (ticks/s)", velTpsHub);
            telemetry.addData("Velocity – Hub (RPM)", tpsToRpm(velTpsHub));
//            telemetry.addData("Velocity – From Position (ticks/s)", velTpsPosDelta);
//            telemetry.addData("Velocity – From Position (RPM)", tpsToRpm(velTpsPosDelta));

            telemetry.addData("Power (Pre-Clamp)", powerPreClamp);
            telemetry.addData("Power (Applied)", powerCmd);

            if (VERBOSE_RC) {
                telemetry.addData("TPR Assumed", TICKS_PER_REV);
                telemetry.addData("Speed Error (ticks/s)", targetTps - velTpsHub);
                telemetry.addData("Speed Error (RPM)", tpsToRpm(targetTps - velTpsHub));
            }
            telemetry.update();
        }

        // --- Optional: dashboard packet ---
        if (SEND_DASH) {
            TelemetryPacket p = new TelemetryPacket();
            p.put("target_tps", targetTps);
            p.put("target_rpm", tpsToRpm(targetTps));
//            p.put("pos_ticks", posTicks);
//            p.put("dpos", dpos);
//            p.put("dt_s", dt);
            p.put("vel_hub_tps", velTpsHub);
            p.put("vel_hub_rpm", tpsToRpm(velTpsHub));
//            p.put("vel_pos_tps", velTpsPosDelta);
//            p.put("vel_pos_rpm", tpsToRpm(velTpsPosDelta));
            p.put("power_pre_clamp", powerPreClamp);
            p.put("power_applied", powerCmd);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    // --- Helpers ---
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private double rpmToTps(double rpm) { return rpm * TICKS_PER_REV / 60.0; }
    private double tpsToRpm(double tps) { return tps * 60.0 / TICKS_PER_REV; }
}
