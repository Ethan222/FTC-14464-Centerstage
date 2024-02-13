package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {
    public RevColorSensorV3 colorSensor;
    public enum Color {
        WHITE, GREEN, PURPLE, YELLOW
    }
    public ColorSensor(HardwareMap hardwareMap, String name) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, name);
    }
    public Color getColor() {
        if(colorSensor.red() > 1000 && colorSensor.green() > 1000 && colorSensor.blue() > 1000)
            return Color.WHITE;
        else if(colorSensor.green() > 600 && colorSensor.red() < 400 && colorSensor.blue() < 400)
            return Color.GREEN;
        else if(colorSensor.blue() > 700 && colorSensor.red() < colorSensor.green() && colorSensor.green() < colorSensor.blue())
            return Color.PURPLE;
        else if(colorSensor.red() > 500 && colorSensor.green() > 500 && colorSensor.blue() < 500)
            return Color.YELLOW;
        else return null;
    }
    public int[] getRGB() {
        return new int[]{
                colorSensor.red(),
                colorSensor.green(),
                colorSensor.blue()
        };
    }
}