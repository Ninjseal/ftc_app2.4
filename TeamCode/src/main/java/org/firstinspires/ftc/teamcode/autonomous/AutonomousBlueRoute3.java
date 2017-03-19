package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousMode;

/**
 * Created by Marius on 3/19/2017.
 */

@Autonomous(name = "AutonomousBlueRoute3", group = "Autonomous")
//@Disabled
public class AutonomousBlueRoute3 extends AutonomousMode {

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
        Throw 1 ball in the center vortex, remove the big ball from center and park the robot = 25 pts

        Move forward 20 CM
        Throw 1 ball in the center vortex
        Move forward 40 CM
         */
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
