package org.firstinspires.ftc.teamcode.motor;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.helpers.control.PIDFController;

@Config
public class Constants {
    public static double depArmS1min = 0.0; //0.41; // 0.4asret
    public static double depArmS1max = 1.0;
    public static double depArmS2min = 0.0;
    public static double depArmS2max = 1.0;//0.585;

    public static PIDFController.PIDCoefficients extendoPID = new PIDFController.PIDCoefficients(0.001, 0.0, 0.0);
    public static PIDFController.PIDCoefficients depositPID = new PIDFController.PIDCoefficients(0.0005, 0.0, 0.0);

}
