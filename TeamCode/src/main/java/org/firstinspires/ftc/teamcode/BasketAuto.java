package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

@Autonomous(name = "BasketAuto2")
//@Disabled
public class BasketAuto2 extends LinearOpMode {

    private final int ARM_LIFT_DOWN = 0;
    private final int ARM_LIFT_UP = 2850;
    private final int ARM_EXTEND_RETRACTED = 0;
    private final int ARM_EXTEND_HORIZ = 435;
    private final int ARM_EXTEND_VERT_SAMPLE = 895;
    private final int ARM_EXTEND_VERT_SPEC_PRE_HANG = 285;
    private final int ARM_EXTEND_VERT_SPEC_POST_HANG = 450;
    private final double CLAW_PITCH_DOWN = 0.85;
    private final double CLAW_PITCH_NEUTRAL = 0.51;
    private final double CLAW_PITCH_UP = 0.17;
    private final double CLAW_YAW_NEUTRAL = 0.48;
    private final double CLAW_OPEN = 0.49;
    private final double CLAW_CLOSED = 0.11;
    // this is only used for teleop
//    private final double CLAW_DUMP_TIME = 0.5;
//    private final double CLAW_CLOSE_TIME = 0.5;
//    private final double CLAW_OPEN_TIME = 0.5;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(-24,-65,Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor armLift = hardwareMap.get(DcMotor.class, "ARMELEV");
        armLift.setDirection(DcMotor.Direction.REVERSE);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor armExtend = hardwareMap.get(DcMotor.class, "ARMEXT");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Servo clawPitch = hardwareMap.get(Servo.class, "PITCH");
        clawPitch.setPosition(CLAW_PITCH_DOWN);

        Servo clawYaw = hardwareMap.get(Servo.class, "YAW");
        clawYaw.setPosition(CLAW_YAW_NEUTRAL);

        Servo claw = hardwareMap.get(Servo.class, "CLAW");
        claw.setPosition(CLAW_CLOSED);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Encoders", "Reset");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // old run actions code
//        Actions.runBlocking(
//                drive.actionBuilder(startPose)
//                        .build()
//        );

        // wip: making some vars for the locations/headings
//        int sample1x = -54;
//        int sample1y = -58;
//        double sample1heading = Math.toRadians(90.0);

        Actions.runBlocking(new SequentialAction(
                // sample 1
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(0.1),
                                new MotorAction(armLift, ARM_LIFT_UP, 10),
                                new MotorAction(armExtend, ARM_EXTEND_VERT_SAMPLE, 10),
                                new ServoAction(clawYaw, CLAW_YAW_NEUTRAL, 0.0),
                                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5)),
                        drive.actionBuilder(new Pose2d(-24,-65,Math.toRadians(90.0)))
                                .lineToY(-61)
                                .strafeToLinearHeading(new Vector2d(-54, -58), Math.toRadians(45.0))
                                .build()
                ),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),
                new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0.0),
                new ParallelAction(
                        new MotorAction(armLift, ARM_LIFT_DOWN, 10),
                        drive.actionBuilder(new Pose2d(-54,-58,Math.toRadians(45.0)))
                                .strafeToLinearHeading(new Vector2d(-47,-39), Math.toRadians(90.0))
                                .build()
                ),

                // sample 2
                new ServoAction(claw, CLAW_CLOSED, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.0),
                new ParallelAction(
                        new SequentialAction(
                                new MotorAction(armLift, ARM_LIFT_UP, 500),
                                new MotorAction(armExtend, ARM_EXTEND_VERT_SAMPLE, 10),
                                new ServoAction(clawYaw, CLAW_YAW_NEUTRAL, 0.0),
                                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5)),
                        drive.actionBuilder(new Pose2d(-47,-39,Math.toRadians(90.0)))
                                .strafeToLinearHeading(new Vector2d(-54, -58), Math.toRadians(45.0))
                                .build()
                ),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),
                new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0.0),
                new ParallelAction(
                        new MotorAction(armLift, ARM_LIFT_DOWN, 10),
                        drive.actionBuilder(new Pose2d(-54,-58,Math.toRadians(45.0)))
                                .strafeToLinearHeading(new Vector2d(-56,-40), Math.toRadians(90.0))
                                .build()
                ),

                // sample 3
                new ServoAction(claw, CLAW_CLOSED, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.0),
                new ParallelAction(
                        new SequentialAction(
                                new MotorAction(armLift, ARM_LIFT_UP, 500),
                                new MotorAction(armExtend, ARM_EXTEND_VERT_SAMPLE, 10),
                                new ServoAction(clawYaw, CLAW_YAW_NEUTRAL, 0.0),
                                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5)),
                        drive.actionBuilder(new Pose2d(-56,-40,Math.toRadians(90.0)))
                                .strafeToLinearHeading(new Vector2d(-53, -59), Math.toRadians(45.0))
                                .build()
                ),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED+271, 40),
                new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0.0),
                new ParallelAction(
                        new SequentialAction(
                                new MotorAction(armLift, ARM_LIFT_DOWN, 10),
                                new MotorAction(armExtend, 393, 10)),
                        drive.actionBuilder(new Pose2d(-53,-59,Math.toRadians(45.0)))
                                .strafeToLinearHeading(new Vector2d(-53, -49), Math.toRadians(119.0))
                                .build()
                )
                ,

                // sample 4
                new ServoAction(claw, CLAW_CLOSED, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.0),
                new ParallelAction(
                        new SequentialAction(
                                new MotorAction(armLift, ARM_LIFT_UP, 500),
                                new MotorAction(armExtend, ARM_EXTEND_VERT_SAMPLE, 10),
                                new ServoAction(clawYaw, CLAW_YAW_NEUTRAL, 0.0),
                                new ServoAction(clawPitch, CLAW_PITCH_UP, 0.5)),
                        drive.actionBuilder(new Pose2d(-57,-39,Math.toRadians(90.0))) // TODO: change actionBuilder Pose2d to 4th sample grabbing position
                                .strafeToLinearHeading(new Vector2d(-51, -57), Math.toRadians(45.0))
                                .build()
                ),
                new ServoAction(claw, CLAW_OPEN, 0.5),
                new ServoAction(clawPitch, CLAW_PITCH_NEUTRAL, 0.5),
                new MotorAction(armExtend, ARM_EXTEND_RETRACTED, 40),
                new MotorAction(armLift, ARM_LIFT_DOWN, 10),
                new ServoAction(clawPitch, CLAW_PITCH_DOWN, 0.0)
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
