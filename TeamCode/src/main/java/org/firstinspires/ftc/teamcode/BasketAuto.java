package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BasketAuto")
public class BasketAuto extends LinearOpMode {

    private final int ARM_LIFT_DOWN = 0;
    private final int ARM_LIFT_UP = 2850;
    private final int ARM_EXTEND_RETRACTED = 0;
    private final int ARM_EXTEND_HORIZ = 1600;
    private final int ARM_EXTEND_VERT_SAMPLE = 3300;
    private final int ARM_EXTEND_VERT_SPEC_PRE_HANG = 1100;
    private final int ARM_EXTEND_VERT_SPEC_POST_HANG = 1700;
    private final double CLAW_PITCH_DOWN = 0.98; // was originally .99
    private final double CLAW_PITCH_NEUTRAL = 0.63;
    private final double CLAW_PITCH_UP = 0.33;
    private final double CLAW_YAW_NEUTRAL = 0.5;
    private final double CLAW_OPEN = 0.66;
    private final double CLAW_CLOSED = 0.27;
    private final double CLAW_DUMP_TIME = 1.0;
    private final double CLAW_CLOSE_TIME = 1.0;
    private final double CLAW_OPEN_TIME = 1.0;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-24,-65,Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor armLift = hardwareMap.get(DcMotor.class, "ARMELEV");
        armLift.setDirection(DcMotor.Direction.REVERSE);

        DcMotor armExtend = hardwareMap.get(DcMotor.class, "ARMEXT");
        armExtend.setDirection(DcMotor.Direction.REVERSE);

        Servo clawPitch = hardwareMap.get(Servo.class, "PITCH");
        clawPitch.setPosition(CLAW_PITCH_DOWN);

        Servo clawYaw = hardwareMap.get(Servo.class, "YAW");
        clawYaw.setPosition(CLAW_YAW_NEUTRAL);

        Servo claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(CLAW_CLOSED);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 1.0))
                        .strafeTo(new Vector2d(-57,-59))
                        .turnTo(Math.toRadians(45.0))
//                        .strafeToLinearHeading(new Vector2d(-58,-61), Math.toRadians(45.0))
                        .stopAndAdd(new MotorAction(armLift, ARM_LIFT_UP, 10))
                        .stopAndAdd(new MotorAction(armExtend, ARM_EXTEND_VERT_SAMPLE, 10))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5))
                        .stopAndAdd(new ServoAction(claw, CLAW_OPEN, 0.5))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5))
                        .stopAndAdd(new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0))
                        .stopAndAdd(new MotorAction(armLift, ARM_LIFT_DOWN, 10))

                        .strafeToLinearHeading(new Vector2d(-47,-38), Math.toRadians(90.0))

                        .stopAndAdd(new ServoAction(claw, CLAW_CLOSED, 0.5))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5))
                        .strafeToLinearHeading(new Vector2d(-57, -59), 45.0)
                        .stopAndAdd(new MotorAction(armLift, ARM_LIFT_UP, 10))
                        .stopAndAdd(new MotorAction(armExtend, ARM_EXTEND_VERT_SAMPLE, 10))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5))
                        .stopAndAdd(new ServoAction(claw, CLAW_OPEN, 0.5))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5))
                        .stopAndAdd(new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40))
                        .stopAndAdd(new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0))
                        .stopAndAdd(new MotorAction(armLift, ARM_LIFT_DOWN, 10))

                        .build()
        );
    }

    public class MotorAction implements Action {
        DcMotor motor;
        int position;
        int margin;
        boolean hasInitialized;

        public MotorAction(DcMotor m, int p, int mar) {
            this.motor = m;
            this.position = p;
            this.margin = mar;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasInitialized) {
                hasInitialized = true;

                motor.setTargetPosition(position);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1.0);
            }
            return (Math.abs(motor.getCurrentPosition() - position) >= margin);
        }
    }

    public class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer;
        double time;

        public ServoAction(Servo s, double p, double t) {
            this.servo = s;
            this.position = p;
            this.time = t;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }
            return timer.seconds() < time;
        }
    }
}
