package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
//@Disabled
public class TestEncoders extends OpMode {
    Encoder frontEncoder;
    Encoder rightEncoder;
    Encoder leftEncoder;

    Encoder.Direction leftEncoderDir = Encoder.Direction.FORWARD;
    Encoder.Direction rightEncoderDir = Encoder.Direction.REVERSE;
    Encoder.Direction frontEncoderDir = Encoder.Direction.FORWARD;

    @Override
    public void init() {
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack")); //no encoder launch1 or launch0 or leftBack or leftFront
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));//works 100%

        frontEncoder.setDirection(frontEncoderDir);
        leftEncoder.setDirection(leftEncoderDir);
        rightEncoder.setDirection(rightEncoderDir);
    }

    @Override
    public void loop() {
        telemetry.addData("Front Encoder", String.valueOf(frontEncoder.getCurrentPosition()));
        telemetry.addData("Left Encoder", String.valueOf(leftEncoder.getCurrentPosition()));
        telemetry.addData("Right Encoder", String.valueOf(rightEncoder.getCurrentPosition()));
    }
}
