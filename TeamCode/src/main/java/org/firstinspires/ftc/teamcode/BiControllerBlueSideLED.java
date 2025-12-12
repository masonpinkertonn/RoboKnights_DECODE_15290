package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DriveCode2026BicontrollerBlue", group="TeleOp")
public class DriveCode2026Bicontroller extends LinearOpMode {

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0015, 0, 0.0000015);

    public static double kV = 0.00042;
    public static double kA = 0.0006;
    public static double kStatic = 0;

    // LED speed threshold - adjust this value to tune when LED turns on
    public static double VELOCITY_THRESHOLD = 0.95; // 95% of target velocity

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();

    SampleMecanumDrive drive;

    // Drive motors
    private DcMotor rightFront, rightBack, leftBack, leftFront;
    // Other motors
    private DcMotor vector, intake;
    // Continuous rotation servo
    private CRServo corner;
    
    // LED servo
    private Servo led;

    DcMotorEx launch0, launch1;

    private Pose2d startPose;

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        startPose = PoseStorage.currentPose;

        drive.setPoseEstimate(startPose);

        // --- Initialize motors ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        vector = hardwareMap.get(DcMotor.class, "vector");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launch0 = hardwareMap.get(DcMotorEx.class, "launch0");
        launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launch0.setDirection(DcMotorSimple.Direction.FORWARD);
        launch1.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- Initialize LED ---
        led = hardwareMap.get(Servo.class, "led");
        // Configure PWM range (optional but recommended)
        PwmControl.PwmRange ledRange = new PwmControl.PwmRange(500, 2500);
        led.setPwmRange(ledRange);
        led.setPosition(0.1); // Start with at red.

        // --- Initialize CRServo ---
        corner = hardwareMap.get(CRServo.class, "corner");

        // Motor directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        corner.setDirection(DcMotor.Direction.REVERSE);

        launch0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        TuningController tuningController = new TuningController();

        double lastTargetVelo = 0.0;
        double lastKv = kV;
        double lastKa = kA;
        double lastKstatic = kStatic;

        double MOTOR_TICKS_PER_REV = 28;
        double MOTOR_GEAR_RATIO = 16.0/24;

        // Stop servo initially
        corner.setPower(0);

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Joystick values
            double driveY = -gamepad1.left_stick_y; // forward/back
            double driveX = gamepad1.left_stick_x;  // strafe
            double turn   = gamepad1.right_stick_x;   // rotation

            // Mecanum drive formula
            double rf = driveY - driveX - turn;
            double rb = driveY + driveX - turn;
            double lf = driveY + driveX + turn;
            double lb = driveY - driveX + turn;

            // Normalize so max is 1
            double max = Math.max(Math.abs(rf), Math.max(Math.abs(rb), Math.max(Math.abs(lf), Math.abs(lb))));
            if (max > 1.0) {
                rf /= max;
                rb /= max;
                lf /= max;
                lb /= max;
            }

            // Set motor powers
            rightFront.setPower(rf);
            rightBack.setPower(rb);
            leftFront.setPower(lf);
            leftBack.setPower(lb);

            // Intake motor control (B button)
            if (gamepad2.b) {
                vector.setPower(1);
                intake.setPower(1);

                // Vector motor control (Right bumper)
            } else if (gamepad2.right_bumper) {
                intake.setPower(1);

                //Corner CRServo control (X button)
            } else if (gamepad2.x) {
                corner.setPower(1);
                intake.setPower(1);
            } else {
                corner.setPower(0);
                intake.setPower(0);
                vector.setPower(0);
            }

            //launch button (Y button)
            if(gamepad2.y){
                double targetVelo = 1850 * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;

                veloController.setTargetVelocity(targetVelo);
                veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
                veloTimer.reset();

                lastTargetVelo = targetVelo;

                telemetry.addData("targetVelocity", targetVelo);

                double motorPos = launch1.getCurrentPosition();
                double motorVelo = launch1.getVelocity();

                double power = veloController.update(motorPos, motorVelo);
                launch0.setPower(power);
                launch1.setPower(power);

                if(lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                    lastKv = kV;
                    lastKa = kA;
                    lastKstatic = kStatic;

                    veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
                }

                // Check if launcher is at speed and control LED
                double velocityRatio = motorVelo / targetVelo;
                if (velocityRatio >= VELOCITY_THRESHOLD) {
                    led.setPosition(0.5); // LED on when at speed
                } elseif (velocityRatio >= (VELOCITY_THRESHOLD / 2) ) {
                    led.setPosition(0.25); 
                } else {
                    led.setPosition(0.1);
                }

                telemetry.addData("velocity", motorVelo);
                telemetry.addData("velocity ratio", velocityRatio);
                telemetry.addData("LED status", velocityRatio >= VELOCITY_THRESHOLD ? "ON" : "OFF");

            }else{
                launch0.setPower(0);
                launch1.setPower(0);
                led.setPosition(0); // LED off when not launching
            }

            if (gamepad1.a || gamepad2.a)
            {
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-25, -25, Math.toRadians(-45)))
                        .build());
                drive.setPoseEstimate(new Pose2d(-25, -25, Math.toRadians(-45)));
            }

            if (gamepad1.dpad_down || gamepad2.dpad_down)
            {
                vector.setPower(-1.0);
                intake.setPower(-1.0);
            }

            if (gamepad1.dpad_up || gamepad2.dpad_up)
            {
                drive.setPoseEstimate(new Pose2d(-25, -25, Math.toRadians(-45)));
            }

            telemetry.addData("Pose:", drive.getPoseEstimate());

            telemetry.update();

            drive.update();

            idle();
        }
    }
}
