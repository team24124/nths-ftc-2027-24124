package org.firstinspires.ftc.teamcode.interfaces;

public interface SubsystemBase {
    /**
     * Method called periodically in OpMode
     * Subsystems running PID loops are required to use periodic() OR can use other methods if they use a PID only partially
     * By default has an empty body, able to be overridden.
     */
    default void periodic() {}

    String getName();
}
