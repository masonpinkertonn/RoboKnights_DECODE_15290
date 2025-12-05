package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp(name="PWM LED Test")
public class PWM_LED_Test extends LinearOpMode {

    private Servo led;  // Using a servo port to output PWM

    @Override
    public void runOpMode() {

        led = hardwareMap.get(Servo.class, "led");

        // OPTIONAL: Configure PWM range (500µs → 2500µs)
        PwmControl.PwmRange ledRange = new PwmControl.PwmRange(500, 2500);
        led.setPwmRange(ledRange);

        waitForStart();

        while (opModeIsActive()) {

            // Example: Brightness controlled by gamepad right trigger
            double brightness = gamepad1.right_trigger; // 0 to 1
            led.setPosition(brightness);

            telemetry.addData("Brightness", brightness);
            telemetry.update();
        }
    }
}
