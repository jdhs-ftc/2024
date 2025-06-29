package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.ColorRangefinder;


@TeleOp
public class ConfigureColorRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf =
                new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "color"));
        /*
        Using this example configuration, you can detect all three sample colors based on which pin is reading true:
        both      --> yellow
        only pin0 --> blue
        only pin1 --> red
        neither   --> no object
         */


        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 180 / 360.0 * 255, 250 / 360.0 * 255); // blue
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 55 / 360.0 * 255, 90 / 360.0 * 255); // yellow
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 100); // 20mm or closer requirement

        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0 / 360.0 * 255, 50 / 360.0 * 255); // red
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 55 / 360.0 * 255, 90 / 360.0 * 255); // yellow
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 100); // 20mm or closer requirement
        /*
        crf.setPin0Analog(ColorRangefinder.AnalogMode.HSV);
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 0, 20); // placeholder
        */
        crf.setLedBrightness(255);

        waitForStart();

        stop();
    }
}
