package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class Intake implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx intake;
    public boolean powered = false;
    public double targetVel = 0;

    public Intake(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, "intake"); // Connected Ehub 0
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public Command runIntake() {
        return instant(() -> {
            targetVel = -((double) 1150/60)*360; // 6900 degrees per second
            intake.setVelocity(targetVel * (145.1/360)); // Degree to tick conversion
            powered = true;
        });
    }

    public Command reverseIntake() {
        return instant(() -> {
            targetVel = ((double) 1150/60)*180; // 3450 degrees per second
            intake.setVelocity(targetVel * (145.1/360)); // Degree to tick conversion
            powered = true;
        });
    }

    public Command stopIntake() {
        return instant(() -> {
            targetVel = 0;
            intake.setVelocity(0);
            powered = false;
        });
    }

    public double velocity() {
        return intake.getVelocity();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake Powered", powered);
        telemetry.addData("Intake Target Velocity (tps)", targetVel * (145.1/360));
        telemetry.addData("Intake Velocity (tps)", velocity());
    }

    @Override
    public String getName() {
        return "Intake";
    }
}
