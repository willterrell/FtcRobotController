package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AbState;
import org.firstinspires.ftc.teamcode.HardwareHandler;
import org.firstinspires.ftc.teamcode.movement.encoder.EncoderMove;
import org.firstinspires.ftc.teamcode.structures.PlaceholderState;
import org.firstinspires.ftc.teamcode.structures.PosType;
import org.firstinspires.ftc.teamcode.structures.TelemetryObj;

@Config
@Autonomous(name="movement test")
public class DistanceTest extends LinearOpMode {
    public static int TARGET_TICKS = 3000;
    public static double KT = 1, KP = 0, POWER = 0.3;
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    public static double KLF = 1, KLR = 0.9825, KRF = 1, KRR = 0.9825;
    public static double KBACK = 0.9895;
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareHandler hardwareHandler = new HardwareHandler(hardwareMap, telemetry);
        waitForStart();
        int distance = (int)(KT * TARGET_TICKS + KP * POWER);
        hardwareHandler.setDriveTrainEncoderTargets((int)(-distance), (int)(distance * KBACK), (int)(distance), (int)(-distance * KBACK));
        hardwareHandler.setDriveTrainPowers(POWER * KLF, POWER * KLR, POWER * KRF, POWER * KRR);

        while(hardwareHandler.driveTrainIsBusy()) {

        }
//        EncoderMove forward1 = new EncoderMove("move1", hardwareHandler, new Position(DistanceUnit.INCH, 0, 10000, 0, 0), 180, 0.5, PosType.RELATIVE);
////        EncoderMove forward2 = new EncoderMove("move2", hardwareHandler, new Position(DistanceUnit.INCH, 0, 12*12, 0, 0), 180, 1, PosType.RELATIVE);
////        EncoderMove forward3 = new EncoderMove("move3", hardwareHandler, new Position(DistanceUnit.INCH, 0, 12*12, 0, 0), 180, 1, PosType.RELATIVE);
//        forward1.putNextState("next", new PlaceholderState());
////        forward2.putNextState("next", forward3);
////        forward3.putNextState("next", new PlaceholderState());
//        AbState currState = forward1;
//        currState.init();
//        hardwareHandler.setDriveTrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            currState.run();
//            currState = currState.next();
//
//            for (TelemetryObj obj : currState.getTelemetries()) {
//                telemetry.addData(obj.getCaption(), obj.getContent());
//            }
//            telemetry.addData("AbState name: ", currState.getName());
//            telemetry.addData("Is busy: ", hardwareHandler.driveTrainIsBusy());
//            telemetry.update();
//        }
    }
}
