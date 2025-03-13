package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SpecAuto")
//@Disabled
public class SpecAuto extends LinearOpMode {


    private final int ARM_LIFT_UP = 2850;
    private final int ARM_EXTEND_RETRACTED = 1;
    private final int ARM_EXTEND_VERT_SPEC_PRE_HANG = 285;
    private final int ARM_EXTEND_VERT_SPEC_POST_HANG = 450;
    private final double CLAW_PITCH_DOWN = 0.85;
    private final double CLAW_PITCH_NEUTRAL = 0.45;
    private final double CLAW_PITCH_UP = 0.17;
    private final double CLAW_YAW_NEUTRAL = 0.48;
    private final double CLAW_OPEN = 0.49;
    private final double CLAW_CLOSED = 0.11;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(9,-65,Math.toRadians(270.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor armLift = hardwareMap.get(DcMotor.class, "ARMELEV");
        armLift.setDirection(DcMotor.Direction.REVERSE);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor armExtend = hardwareMap.get(DcMotor.class, "ARMEXT");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Servo clawPitch = hardwareMap.get(Servo.class, "PITCH");
        clawPitch.setPosition(CLAW_PITCH_UP);

        Servo clawYaw = hardwareMap.get(Servo.class, "YAW");
        clawYaw.setPosition(CLAW_YAW_NEUTRAL);

        Servo claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(CLAW_CLOSED);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Encoders", "Reset");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                //Deposit spec 1
                new SequentialAction(
                        new SleepAction(0.1),
                        new MotorAction(armLift, ARM_LIFT_UP, 10),
                        new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_PRE_HANG, 10),
                        drive.actionBuilder(new Pose2d(9,-65,Math.toRadians(270.0)))
                                .strafeToLinearHeading(new Vector2d(5,-30), Math.toRadians(270.0))
                                .build()
                ),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_POST_HANG, 10),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0.5)
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),

                //Drive to spec 2
                drive.actionBuilder(new Pose2d(5,-34,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(33, -34), Math.toRadians(270.0))
                        .build(),
                drive.actionBuilder(new Pose2d(33,-34,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(33, -14), Math.toRadians(270.0))
                        .build(),
                drive.actionBuilder(new Pose2d(33,-14,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(44, -14), Math.toRadians(270.0))
                        .build(),

                //Push spec 2 to human player
                drive.actionBuilder(new Pose2d(44,-14,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(46, -52), Math.toRadians(270.0))
                        .build(),

                //Drive to spec 3
                drive.actionBuilder(new Pose2d(46,-52,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(46, -14), Math.toRadians(270.0))
                        .build(),
                drive.actionBuilder(new Pose2d(46,-14,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(54, -14), Math.toRadians(270.0))
                        .build(),

                //Push spec 3 to human player
                drive.actionBuilder(new Pose2d(54,-14,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(52, -52), Math.toRadians(270.0))
                        .build(),

                //Deposit spec 2
                drive.actionBuilder(new Pose2d(52,-52,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(270.0))
                        .build(),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5),
                new ServoAction(claw, CLAW_CLOSED, 0.5),
                new MotorAction(armLift, ARM_LIFT_UP, 10),
                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_PRE_HANG, 10),
                drive.actionBuilder(new Pose2d(40, -58,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(7,-30), Math.toRadians(270.0))
                        .build(),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_POST_HANG, 10),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5)
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),

                //Deposit spec 3
                drive.actionBuilder(new Pose2d(7, -30,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(270.0))
                        .build(),
                new ServoAction(claw, CLAW_CLOSED, 0.5),
                new MotorAction(armLift, ARM_LIFT_UP, 10),
                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_PRE_HANG, 10),
                drive.actionBuilder(new Pose2d(40, -58,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(9,-30), Math.toRadians(270.0))
                        .build(),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_POST_HANG, 10),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5)
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),

                //Deposit Spec 4 (Pre-loadable)
                drive.actionBuilder(new Pose2d(9, -30,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(270.0))
                        .build(),
                new ServoAction(claw, CLAW_CLOSED, 0.5),
                new MotorAction(armLift, ARM_LIFT_UP, 10),
                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_PRE_HANG, 10),
                drive.actionBuilder(new Pose2d(40, -58,Math.toRadians(270.0)))
                        .strafeToLinearHeading(new Vector2d(11,-30), Math.toRadians(270.0))
                        .build(),
                new MotorAction(armExtend, ARM_EXTEND_VERT_SPEC_POST_HANG, 10),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5)
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),
        ));
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
