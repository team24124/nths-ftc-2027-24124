package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Simple static field serving as a storage medium for the bot's pose
 * This allows different classes/OpModes to set and read from a central source of truth
 * A static field allows data to persist between OpModes
 */
public class PoseStorage {
    public static Pose currentPose = new Pose(0,0,0);

    public enum Alliance { RED, BLUE }
    public static Alliance currentAlliance = Alliance.BLUE;
}