package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveTeleOpConfig {
    // Drive tuning exposed to the dashboard
    public static double Kp = 0.1, Kd = 0.002, Ki = 2, Kf = 0.5, Kl = 0.5;
    public static double module1Adjust = -10, module2Adjust = -10, module3Adjust = -45;
    public static double driveScale = 1.0, rotationScale = 1.0;

    // Turret tuning
    public static double yawScale = 0.5;
}
