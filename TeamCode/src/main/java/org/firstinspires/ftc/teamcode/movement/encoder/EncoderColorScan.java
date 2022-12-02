package org.firstinspires.ftc.teamcode.movement.encoder;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.imu.RotateWithIMU;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;

import java.util.HashMap;

public class EncoderColorScan extends AbState {
    private HardwareHandler hardwareHandler;
    private AbState currState;
    public double[] colorReading;
    public EncoderColorScan(String name, HardwareHandler hardwareHandler) {
        super(name, "pink", "green", "purple");
        this.hardwareHandler = hardwareHandler;

    }


    public void init(){
        colorReading = HardwareHandler.getColorSensorReading();
    }

    @Override
    public AbState next(HashMap<String, AbState> nextStateMap) {
        if (colorReading[0]<150 && colorReading[2]<150 && colorReading[1]<25){
            return getNextState("movePurple");
        }
        else if (colorReading[0]>150 && colorReading[1]>200){
            return getNextState("green");
        }
        else {
            return getNextState("movePink");
        }



    }

    @Override
    public void run() {
        // save color_sensor.argb

    }
}



