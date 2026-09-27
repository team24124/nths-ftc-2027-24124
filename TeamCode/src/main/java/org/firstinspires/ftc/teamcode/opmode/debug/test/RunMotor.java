package org.firstinspires.ftc.teamcode.opmode.debug.test;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor", group = "test")
public class RunMotor extends OpMode {
    private GamepadEx driver;
    private DcMotor motor;
    double power = 0;

    @Override
    public void init() {
        driver = new GamepadEx(gamepad1);
        motor = hardwareMap.get(DcMotorEx.class, "motor");
    }

    @Override
    public void loop() {
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            power -= 0.1;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            power += 0.1;
        }
        motor.setPower(power);
        driver.readButtons();

        telemetry.addData("power", power);
        telemetry.update();
    }
}