package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousMode;

/**
 * Created by Marius on 3/19/2017.
 */

@Autonomous(name = "AutonomousRedRoute4", group = "Autonomous")
//@Disabled
public class AutonomousRedRoute4 extends AutonomousMode {

    @Override
    protected void initOpMode() throws InterruptedException {
        initHardware();
    }

    protected void runOp() throws InterruptedException {
        while (!isStopRequested() && gyroSensor.isCalibrating()) {
            sleep(50);
            idle();
        }

        // Init wheels motors
        leftMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        Capture 1 beacon, throw 1 ball in the center vortex, remove the big ball from center and park the robot = 55 pts

        Move forward 10 CM
        Turn left 45 degrees
        Move forward until ods detects the line
        Follow the line until the distance between robot and wall is <= 17 CM
        Detect the color of beacon and capture it
        Move backwards 15 CM
        Throw 1 ball in the center vortex
        Move backwards 100 CM
         */
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
