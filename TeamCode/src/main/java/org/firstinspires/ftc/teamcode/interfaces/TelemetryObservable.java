package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;

public interface TelemetryObservable {
    /**
     * Method called by TelemetryControl {@link TelemetryControl} to update corresponding telemetry
     * @param telemetry Telemetry system to use
     */
    void updateTelemetry(Telemetry telemetry);

    default String getName(){
        if(this instanceof SubsystemBase) return this.getName();

        return ""; // If the observable object is not a subsystem don't give it any name
    }
}
