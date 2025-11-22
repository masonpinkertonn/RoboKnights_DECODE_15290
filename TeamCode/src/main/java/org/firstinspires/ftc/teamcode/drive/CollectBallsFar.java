package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TuningController;
import org.firstinspires.ftc.teamcode.VelocityPIDFController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Blue - Far Auto Ball Collection")
public class CollectBallsFar extends OpMode {
    DcMotor intake;
    DcMotor vector;

    public static double firstTime = 0;
    public static double secondTime = 7;
    public static double thirdTime = 15;
    public static double fourthTime = 20;
    public static double waitTime = 4;



    CRServo corner;

    SampleMecanumDrive drive;

    Pose2d start;

    TrajectorySequence mySequence;

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0015, 0, 0.0000015);

    public static double kV = 0.00042;
    public static double kA = 0.0006;
    public static double kStatic = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();

    private DcMotorEx launch0, launch1;

    VelocityPIDFController veloController;

    TuningController tuningController;

    double lastKv;
    double lastKa;
    double lastKstatic;
    double MOTOR_TICKS_PER_REV;
    double MOTOR_GEAR_RATIO;

    double lastTargetVelo = 0.0;

    ElapsedTime timer;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        vector = hardwareMap.get(DcMotor.class, "vector");

        launch0 = hardwareMap.get(DcMotorEx.class, "launch0");
        launch1 = hardwareMap.get(DcMotorEx.class, "launch1");

        launch0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        //boolean runMotor = false;

        veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        tuningController = new TuningController();


        lastKv = kV;
        lastKa = kA;
        lastKstatic = kStatic;

        MOTOR_TICKS_PER_REV = 28;
        //double MOTOR_MAX_RPM = 6000;
        MOTOR_GEAR_RATIO = 10.0/14;

        //launch0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //launch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        corner = hardwareMap.get(CRServo.class, "corner");

        corner.setDirection(DcMotorSimple.Direction.REVERSE);

        launch0.setDirection(DcMotorSimple.Direction.FORWARD);
        launch1.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        drive = new SampleMecanumDrive(hardwareMap);

        start = new Pose2d(62, -25, Math.toRadians(-90));

        drive.setPoseEstimate(start);

        mySequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-40)))
                .addTemporalMarker(() -> {

                    //launch0.setPower(0.65);
                    //launch1.setPower(0.65);
                    corner.setPower(1.0);
                    vector.setPower(1.0);
                    intake.setPower(1.0);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    intake.setPower(0.0);
                    //launch0.setPower(0.0);
                    //launch1.setPower(0.0);
                    corner.setPower(0.0);
                    vector.setPower(0.0);
                })
                //.lineTo(new Vector2d(10, 0))
                //.turn(Math.toRadians(-135))
                /*.splineTo(new Vector2d(11.75, -28), Math.toRadians(-90))
                .forward(50)
                .waitSeconds(50)*/
                .lineToLinearHeading(new Pose2d(37.25, -28, Math.toRadians(-90)))
                /*.addTemporalMarker(() -> {
                    intake.setPower(0.6);
                    launch0.setPower(1.0);
                    launch1.setPower(1.0);
                    corner.setPower(1.0);
                })
                .waitSeconds(7.5)
                .addTemporalMarker(() -> {
                    intake.setPower(0.0);
                    launch0.setPower(0.0);
                    launch1.setPower(0.0);
                    corner.setPower(0.0);
                })*/
                /*.addTemporalMarker(6, () -> {
                    launch0.setPower(1.0);
                    launch1.setPower(1.0);
                })
                .addTemporalMarker(20, () -> {
                    launch0.setPower(1.0);
                    launch1.setPower(1.0);
                })*/
                //.lineToLinearHeading(new Pose2d(-11.75, -28, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    intake.setPower(1.0);
                    vector.setPower(1.0);
                })
                //.waitSeconds(0.5)
                .forward(25)
                .addDisplacementMarker(() -> {
                    intake.setPower(0.0);
                    vector.setPower(0.0);
                })
                //.back(25)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-30)))
                //.turn(Math.toRadians(-45))
                .addTemporalMarker(() -> {

                    //launch0.setPower(0.65);
                    //launch1.setPower(0.65);
                    corner.setPower(1.0);
                    vector.setPower(1.0);
                    intake.setPower(1.0);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    intake.setPower(0.0);
                    //launch0.setPower(0.0);
                    //launch1.setPower(0.0);
                    corner.setPower(0.0);
                    vector.setPower(0.0);
                })
                //.turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(11.75, -28, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    intake.setPower(1.0);
                    vector.setPower(1.0);
                })
                //.waitSeconds(0.1)
                .forward(26)
                .addDisplacementMarker(() -> {
                    intake.setPower(0.0);
                    vector.setPower(0.0);
                })
                //.back(25)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-45)))
                //.turn(Math.toRadians(-45))
                .addTemporalMarker(() -> {

                    //launch0.setPower(1.0);
                    //launch1.setPower(1.0);
                    corner.setPower(1.0);
                    vector.setPower(1.0);
                    intake.setPower(1.0);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    intake.setPower(0.0);
                    //launch0.setPower(0.0);
                    //launch1.setPower(0.0);
                    corner.setPower(0.0);
                    vector.setPower(0.0);
                })
                //.turn(Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(37.25, -28, Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    intake.setPower(1.0);
                    vector.setPower(1.0);
                })
                //.waitSeconds(0.1)
                .forward(30)
                .addDisplacementMarker(() -> {
                    intake.setPower(0.0);
                    vector.setPower(0.0);
                })
                //.back(25)
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-45)))
                //.turn(Math.toRadians(-45))
                .addTemporalMarker(() -> {

                    //launch0.setPower(1.0);
                    //launch1.setPower(1.0);
                    corner.setPower(1.0);
                    vector.setPower(1.0);
                    intake.setPower(1.0);
                })
                .waitSeconds(4)
                .addTemporalMarker(() -> {
                    intake.setPower(0.0);
                    //launch0.setPower(0.0);
                    //launch1.setPower(0.0);
                    corner.setPower(0.0);
                    vector.setPower(0.0);
                })
                .build();

        drive.followTrajectorySequenceAsync(mySequence);

        veloTimer.reset();

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        drive.update();
        //drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.getPoseEstimate();





        double targetVelo = 2300 * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;

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

        if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
            lastKv = kV;
            lastKa = kA;
            lastKstatic = kStatic;

            veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        }
        telemetry.addData("velocity", motorVelo);




    }
}
