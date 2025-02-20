package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="EncoderTest")
public class EncoderTest extends LinearOpMode {


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

        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLift = hardwareMap.get(DcMotor.class, "ARMELEV");
        armLift.setDirection(DcMotor.Direction.REVERSE);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armExtend = hardwareMap.get(DcMotor.class, "ARMEXT");
        armExtend.setDirection(DcMotor.Direction.REVERSE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawPitch = hardwareMap.get(Servo.class, "PITCH");

        clawYaw = hardwareMap.get(Servo.class, "YAW");

        claw = hardwareMap.get(Servo.class, "CLAW");
//        claw.setPosition(0.69);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        armTimer.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            claw.setPosition(0.27);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("armLift", armLift.getCurrentPosition());
            telemetry.addData("armExtend", armExtend.getCurrentPosition());
            telemetry.addData("par", rightBackDrive.getCurrentPosition());
            telemetry.addData("perp", leftFrontDrive.getCurrentPosition());
            telemetry.addData("a", claw.getPosition());
            telemetry.update();
        }
    }
}
