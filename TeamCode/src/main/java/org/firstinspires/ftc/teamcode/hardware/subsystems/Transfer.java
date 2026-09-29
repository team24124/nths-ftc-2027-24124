package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class Transfer implements SubsystemBase, TelemetryObservable {
    public final DcMotorEx transfer;
    public final Servo pollenGate;
    public final Servo nectarGate;
    public boolean powered = false;
    public double targetVel = 0;

    public Transfer(HardwareMap hw) {
        transfer = hw.get(DcMotorEx.class, "transfer"); // Connected Ehub 1
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pollenGate = hw.get(Servo.class, "pollen gate");
        nectarGate = hw.get(Servo.class, "nectar gate");
    }

    public Command runTransfer() {
        return instant(() -> {
            targetVel = -((double) 1150/60)*360; // 6900 degrees per second
            transfer.setVelocity(targetVel * (145.1/360)); // Degree to tick conversion
            powered = true;
        });
    }

    public Command reverseTransfer() {
        return instant(() -> {
            targetVel = ((double) 1150/60)*180; // 3450 degrees per second
            transfer.setVelocity(targetVel * (145.1/360)); // Degree to tick conversion
            powered = true;
        });
    }

    public Command stopTransfer() {
        return instant(() -> {
            targetVel = 0;
            transfer.setVelocity(0);
            powered = false;
        });
    }

    public Command openGates() {
        return instant(() -> {
            pollenGate.setPosition(0);
            nectarGate.setPosition(1);
        });
    }

    public Command closeGates() {
        return instant(() -> {
            pollenGate.setPosition(1);
            nectarGate.setPosition(0);
        });
    }

    public double velocity() {
        return transfer.getVelocity();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer Powered", powered);
        telemetry.addData("Transfer Velocity (tps)", velocity());
        telemetry.addData("Servo State", pollenGate.getPosition());
    }

    @Override
    public String getName() {
        return "Transfer";
    }
}
