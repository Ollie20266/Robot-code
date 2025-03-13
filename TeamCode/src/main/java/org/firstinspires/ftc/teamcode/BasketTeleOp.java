package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="BasketTeleOp")
public class BasketTeleOp extends LinearOpMode {

    private enum ArmState {
        START,                  // starting position, arm down, retracted, claw down, claw open
        LIFT_SAMPLE_DROP_SPEC,  // lift up arm to reach high basket, or drop sample in observation zone
        EXTEND_SAMPLE,          // extend arm to reach high basket
        DUMP_SAMPLE,            // rotate the claw back to face the high basket
        OPEN_SAMPLE,            // open the claw to drop sample in high basket
        UNDUMP_SAMPLE,          // move the claw pitch to neutral position
        RETRACT_SAMPLE,         // retract arm after scoring high basket
        LOWER_SAMPLE,           // lower arm after scoring high basket, return to START
        GRAB_SPEC_OR_RETRACT,   // grab spec from wall or return to START to pick up more samples
        LIFT_SPEC,              // lift spec to hanging position
        EXTEND_SPEC,            // extend to arm to hang spec
        ATTACH_SPEC,            // attach spec to high chamber
        LET_GO_SPEC,            // open claw to let go of spec
        RETRACT_SPEC,           // retract arm after hanging spec
        LOWER_SPEC              // lower arm and pitch claw to neutral after hanging spec (return to GRAB_SPEC_OR_RETRACT
    }

    private ArmState armState = ArmState.START;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime armTimer = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armLift = null;
    private DcMotor armExtend = null;
    private Servo clawPitch = null;
    private Servo clawYaw = null;
    private Servo claw = null;

    private final double DRIVE_POWER = 0.7;
    private final double DRIVE_POWER_LOW = 0.3;
    private final double DRIVE_POWER_HIGH = 1.0;
    private final double ARM_EXTEND_MANUAL_POWER = 0.3;

    private final int ARM_LIFT_DOWN = 0;
    private final int ARM_LIFT_UP = 2850;
    private final int ARM_EXTEND_RETRACTED = 1;
    private final int ARM_EXTEND_HORIZ = 435;
    private final int ARM_EXTEND_VERT_SAMPLE = 895;
    private final int ARM_EXTEND_VERT_SPEC_PRE_HANG = 285;
    private final int ARM_EXTEND_VERT_SPEC_POST_HANG = 450;
    private final double CLAW_PITCH_DOWN = 0.85;
    private final double CLAW_PITCH_NEUTRAL = 0.45;
//    private final double CLAW_PITCH_GRAB_SPEC = 0.51;
    private final double CLAW_PITCH_UP = 0.17;
    private final double CLAW_YAW_NEUTRAL = 0.48;
    private final double CLAW_OPEN = 0.49;
    private final double CLAW_CLOSED = 0.11;
    private final double CLAW_DUMP_TIME = 0.5;
    private final double CLAW_CLOSE_TIME = 0.5;
    private final double CLAW_OPEN_TIME = 0.5;

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "BL");
//        leftBackDrive  = hardwareMap.get(DcMotor.class, "FL");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "BR");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "FR");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armLift = hardwareMap.get(DcMotor.class, "ARMELEV");
        armLift.setDirection(DcMotor.Direction.REVERSE);
        armLift.setTargetPosition(ARM_LIFT_DOWN);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armExtend = hardwareMap.get(DcMotor.class, "ARMEXT");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawPitch = hardwareMap.get(Servo.class, "PITCH");
        clawYaw = hardwareMap.get(Servo.class, "YAW");
        claw = hardwareMap.get(Servo.class, "CLAW");

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        armTimer.reset();

        clawPitch.setPosition(CLAW_PITCH_DOWN);
        clawYaw.setPosition(CLAW_YAW_NEUTRAL);
        claw.setPosition(CLAW_OPEN);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armLift.setPower(1.0);

            switch (armState) {
                case START:
                    armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    double armExtendPower = -gamepad2.left_stick_y;
                    clawYaw.setPosition(CLAW_YAW_NEUTRAL - gamepad2.right_stick_x/3.3);
                    if ((armExtend.getCurrentPosition() - ARM_EXTEND_RETRACTED <= 10
                            && armExtendPower < 0)
                            || (armExtend.getCurrentPosition() >= ARM_EXTEND_HORIZ
                            && armExtendPower > 0)) {
                        armExtendPower = 0.0;
                    }
                    armExtend.setPower(armExtendPower * ARM_EXTEND_MANUAL_POWER);

                    if (gamepad2.right_bumper) {
                        claw.setPosition(CLAW_CLOSED);
                    } else if (gamepad2.left_bumper) {
                        claw.setPosition(CLAW_OPEN);
                    } else if (gamepad2.a) {
                        clawYaw.setPosition(CLAW_YAW_NEUTRAL);
                        clawPitch.setPosition(CLAW_PITCH_NEUTRAL);
                        armExtend.setTargetPosition(ARM_EXTEND_RETRACTED);
                        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armExtend.setPower(1.0);
                        armState = ArmState.LIFT_SAMPLE_DROP_SPEC;
                    }
                    break;
                case LIFT_SAMPLE_DROP_SPEC:
                    if (gamepad2.b) {
                        armLift.setTargetPosition(ARM_LIFT_UP);
                        clawYaw.setPosition(CLAW_YAW_NEUTRAL);
                        armState = ArmState.EXTEND_SAMPLE;
                    } else if (gamepad2.left_bumper) {
                        claw.setPosition(CLAW_OPEN);
                        armState = ArmState.GRAB_SPEC_OR_RETRACT;
                    }
                    break;
                case EXTEND_SAMPLE:
                    if (Math.abs(armLift.getCurrentPosition() - ARM_LIFT_UP) < 10) {
                        armExtend.setTargetPosition(ARM_EXTEND_VERT_SAMPLE);
                        armState = ArmState.DUMP_SAMPLE;
                    }
                    break;
                case DUMP_SAMPLE:
                    if (Math.abs(armExtend.getCurrentPosition() - ARM_EXTEND_VERT_SAMPLE) < 20) {
                        clawPitch.setPosition(CLAW_PITCH_UP);
                        armState = ArmState.OPEN_SAMPLE;
                    }
                    break;
                case OPEN_SAMPLE:
                    if (gamepad2.left_bumper) {
                        claw.setPosition(CLAW_OPEN);
                        armTimer.reset();
                        armState = ArmState.UNDUMP_SAMPLE;
                    }
                    break;
                case UNDUMP_SAMPLE:
                    if (armTimer.seconds() >= CLAW_OPEN_TIME) {
                        clawPitch.setPosition(CLAW_PITCH_NEUTRAL);
                        armTimer.reset();
                        armState = ArmState.RETRACT_SAMPLE;
                    }
                case RETRACT_SAMPLE:
                    if (armTimer.seconds() >= CLAW_DUMP_TIME) {
                        armExtend.setTargetPosition(ARM_EXTEND_RETRACTED);
                        armState = ArmState.LOWER_SAMPLE;
                    }
                    break;
                case LOWER_SAMPLE:
                    if (Math.abs(armExtend.getCurrentPosition() - ARM_EXTEND_RETRACTED) <= 45) {
                        armLift.setTargetPosition(ARM_LIFT_DOWN);
                        clawPitch.setPosition(CLAW_PITCH_DOWN);
                        armState = ArmState.START;
                    }
                    break;
                case GRAB_SPEC_OR_RETRACT:
                    if (gamepad2.x) {
                        clawPitch.setPosition(CLAW_PITCH_DOWN);
                        armState = ArmState.START;
                    } else if (gamepad2.right_trigger > 0.5) {
                        claw.setPosition(CLAW_CLOSED);
                        armTimer.reset();
                        armState = ArmState.LIFT_SPEC;
                    }
                    break;
                case LIFT_SPEC:
                    if (armTimer.seconds() >= CLAW_CLOSE_TIME) {
                        armLift.setTargetPosition(ARM_LIFT_UP);
                        clawPitch.setPosition(CLAW_PITCH_UP);
                        armState = ArmState.EXTEND_SPEC;
                    }
                    break;
                case EXTEND_SPEC:
                    if (Math.abs(armLift.getCurrentPosition() - ARM_LIFT_UP) <= 20) {
                        clawYaw.setPosition(CLAW_YAW_NEUTRAL);
                        armExtend.setTargetPosition(ARM_EXTEND_VERT_SPEC_PRE_HANG);
                        armState = ArmState.ATTACH_SPEC;
                    }
                    break;
                case ATTACH_SPEC:
                    if (gamepad2.left_trigger > 0.5) {
                        armExtend.setTargetPosition(ARM_EXTEND_VERT_SPEC_POST_HANG);
                        armState = ArmState.LET_GO_SPEC;
                    }
                case LET_GO_SPEC:
                    if (Math.abs(armExtend.getCurrentPosition() - ARM_EXTEND_VERT_SPEC_POST_HANG) <= 20) {
                        claw.setPosition(CLAW_OPEN);
                        armTimer.reset();
                        armState = ArmState.RETRACT_SPEC;
                    }
                    break;
                case RETRACT_SPEC:
                    if (armTimer.seconds() >= CLAW_OPEN_TIME) {
                        armExtend.setTargetPosition(ARM_EXTEND_RETRACTED);
                        armState = ArmState.LOWER_SPEC;
                    }
                    break;
                case LOWER_SPEC:
                    if (Math.abs(armExtend.getCurrentPosition() - ARM_EXTEND_RETRACTED) <= 20) {
                        armLift.setTargetPosition(ARM_LIFT_DOWN);
                        clawPitch.setPosition(CLAW_PITCH_NEUTRAL);
                        armState = ArmState.GRAB_SPEC_OR_RETRACT;
                    }
                    break;
                default:
                    // should never be reached, as armState should never be null
                    armState = ArmState.START;
                    break;
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            double drivePower;

            if (gamepad1.left_trigger > 0.5) {
                drivePower = DRIVE_POWER_LOW;
            } else if (gamepad1.right_trigger > 0.5) {
                drivePower = DRIVE_POWER_HIGH;
            } else {
                drivePower = DRIVE_POWER;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * drivePower);
            rightFrontDrive.setPower(rightFrontPower * drivePower);
            leftBackDrive.setPower(leftBackPower * drivePower);
            rightBackDrive.setPower(rightBackPower * drivePower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("armLift", armLift.getCurrentPosition());
            telemetry.addData("armExtend", armExtend.getCurrentPosition());
            telemetry.addData("clawYaw", clawYaw.getPosition());
            telemetry.update();
        }
    }
}
