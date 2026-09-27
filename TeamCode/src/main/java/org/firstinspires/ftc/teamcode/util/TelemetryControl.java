package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

import java.util.ArrayList;

public class TelemetryControl {
    private final MultipleTelemetry telemetry;
    private final ArrayList<TelemetryObservable> observables;

    public TelemetryControl(Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        observables = new ArrayList<>();
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }

    public void update(){
        for(TelemetryObservable subject : observables) {
            telemetry.addLine(subject.getName().toUpperCase());
            subject.updateTelemetry(telemetry);
            telemetry.addLine();
        }
        telemetry.update();
    }

    /**
     * Subscribes a class to the routine telemetry update
     * @param observable A class which implements the TelemetryObservable class
     * @return Returns the TelemetryControl in order to chain methods
     */
    public TelemetryControl subscribe(TelemetryObservable observable) {
        telemetry.addLine(observable.getName() + " has been subscribed to telemetry.");
        observables.add(observable);
        return this;
    }

    /**
     * Unsubscribes a class from the routine telemetry update
     * @param subscribed A class which implements the TelemetryObservable class and is subscribed to the current Telemetry
     */
    public void unsubscribe(TelemetryObservable subscribed){
        telemetry.addLine(subscribed.getName() + " has been unsubscribed from telemetry.");
        observables.remove(subscribed);
    }

    /**
     * Unsubscribes all classes currently subscribe from the routine telemetry update
     */
    public void unsubscribeAll(){
        telemetry.addLine("All subjects unsubscribed from telemetry.");
        observables.clear();
    }
}
