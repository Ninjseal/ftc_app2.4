package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Marius on 3/19/2017.
 */

@Autonomous(name = "AutonomousTest", group = "Autonomous")
//@Disabled
public class AutonomousTest extends AutonomousMode {

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

        // Move forward 10 cm keeping 0 degrees
        gyroDrive(FORWARD_SPEED, 10, 0);
        telemetry.addData("Done!", "Exiting...");
        telemetry.update();
        sleep(500);
    }

    protected void exitOpMode() throws InterruptedException {
        stopMotors();
    }
}
