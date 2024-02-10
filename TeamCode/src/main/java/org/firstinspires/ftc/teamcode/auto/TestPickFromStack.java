package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.auto.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

@Autonomous(name = "Pick from stack test", group = "test")
public class TestPickFromStack extends LinearOpMode {
    private Robot robot;
    private ScheduledExecutorService executorService;
    private ElapsedTime timer;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        executorService = Executors.newSingleThreadScheduledExecutor();
        timer = new ElapsedTime();
        robot.outtake.rotator.retractFully();
        robot.claw2.down();
        robot.claw1.up();
        robot.intake.lower();
        waitForStart();
        pickFromStack();
        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive());
    }
    private void pickFromStack() {
        robot.intake.in();
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .strafeRight(10, MecanumDrive.getVelConstraint(.2*DriveConstants.MAX_VEL), MecanumDrive.getAccelConstraint())
                .build());
        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive());
        robot.intake.raise();
        robot.drive.followTrajectorySequence(robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .forward(10)
                .build());
        while(timer.seconds() < 4 && opModeIsActive());
        robot.intake.stop();
        robot.claw1.down();
    }
}