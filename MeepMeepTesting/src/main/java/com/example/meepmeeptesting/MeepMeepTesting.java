package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double waitTime = 1;
        double spikeMarkBackDistance = 5, backdropBackDistance = 4;
        double slowSpeed = 5;

        // blue
        Pose2d blueBackdrop = new Pose2d(44, 37, 0);
        Pose2d blueStack = new Pose2d(-63, 23, 0);
        Vector2d blueGate = new Vector2d(-6, 14);

        // blue back
        Pose2d blueBackStart = new Pose2d(12, 62.5, -Math.PI / 2);
        Pose2d blueBackPark = new Pose2d(60, 62, 0);

        RoadRunnerBotEntity blueBackLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueBackStart)
                                .splineToSplineHeading(new Pose2d(22, 40, -Math.PI / 2), -Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(blueBackdrop, blueBackdrop.getHeading())
                                .forward(5,
                                        SampleMecanumDrive.getVelocityConstraint(slowSpeed, Math.PI, 15),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineTo(blueGate, Math.PI)
                                .splineToSplineHeading(blueStack, Math.PI)
                                .back(5,
                                        SampleMecanumDrive.getVelocityConstraint(slowSpeed, Math.PI, 15),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .waitSeconds(waitTime)
                                .splineTo(blueGate, 0)
                                .splineToSplineHeading(blueBackdrop, 0)
                                .forward(5,
                                        SampleMecanumDrive.getVelocityConstraint(slowSpeed, Math.PI, 15),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .back(backdropBackDistance)
                                .splineToLinearHeading(blueBackPark, blueBackPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity blueBackCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueBackStart)
                                .splineToSplineHeading(new Pose2d(15, 34, -Math.PI / 2), -Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(blueBackdrop, blueBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToSplineHeading(blueBackPark, blueBackPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity blueBackRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueBackStart)
                                .splineToSplineHeading(new Pose2d(20, 39, -3 * Math.PI / 4), -3 * Math.PI / 4)
                                .splineToSplineHeading(new Pose2d(11, 32, Math.PI), Math.PI)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(blueBackdrop, blueBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToSplineHeading(blueBackPark, blueBackPark.getHeading())
                                .build()
                );

        // blue front
        Pose2d blueFrontStart = new Pose2d(-36, 62, -Math.PI / 2);
        Pose2d blueFrontPark = new Pose2d(56, 14, 0);

        RoadRunnerBotEntity blueFrontLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueFrontStart)
                                //.splineToSplineHeading(new Pose2d(-20, 39, -Math.PI / 4), -Math.PI / 4)
                                .forward(10)
                                .splineToSplineHeading(new Pose2d(-28, 30, 0), 0)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(blueStack, Math.PI)
                                .waitSeconds(waitTime)
                                .splineTo(blueGate, 0)
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(blueBackdrop, blueBackdrop.getHeading())
                                .forward(5,
                                        SampleMecanumDrive.getVelocityConstraint(slowSpeed, Math.PI, 15),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToLinearHeading(blueFrontPark, blueFrontPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity blueFrontCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueFrontStart)
                                .splineToSplineHeading(new Pose2d(-15, 21, -Math.PI / 2), -Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineTo(blueGate, 0)
                                .splineToSplineHeading(blueBackdrop, blueBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(blueFrontPark, blueFrontPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity blueFrontRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(30, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueFrontStart)
                                .splineToSplineHeading(new Pose2d(-45, 27, Math.PI), -Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .lineTo(blueGate.plus(new Vector2d(-10, 0)))
                                .splineTo(blueGate.plus(new Vector2d(10, 0)), 0)
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(blueBackdrop, blueBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(blueFrontPark, blueFrontPark.getHeading())
                                .build()
                );

        // red
        Pose2d redBackdrop = new Pose2d(44, -35, 0);
        Pose2d redStack = new Pose2d(-58, -35, 0);
        Vector2d redGate = new Vector2d(-6, -11);

        // red back
        Pose2d redBackStart = new Pose2d(12, -61, Math.PI / 2);
        Pose2d redBackPark = new Pose2d(56, -61, 0);
        RoadRunnerBotEntity redBackLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redBackStart)
                                //.splineToSplineHeading(new Pose2d(16, -47, 3 * Math.PI / 4), 3 * Math.PI / 4)
                                .forward(10)
                                .splineToSplineHeading(new Pose2d(10, -26, Math.PI), Math.PI)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(redBackdrop, redBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToLinearHeading(redBackPark, redBackPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity redBackCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redBackStart)
                                .splineToSplineHeading(new Pose2d(17, -32, Math.PI / 2), Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(redBackdrop, redBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToSplineHeading(redBackPark, redBackPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity redBackRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redBackStart)
                                .splineToSplineHeading(new Pose2d(24.5, -42, Math.PI / 2), Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(redBackdrop, redBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToSplineHeading(redBackPark, redBackPark.getHeading())
                                .build()
                );

        // red front
        Pose2d redFrontStart = new Pose2d(-36, -61, Math.PI / 2);
        Pose2d redFrontPark = new Pose2d(60, -11, 0);

        RoadRunnerBotEntity redFrontLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redFrontStart)
                                .splineToSplineHeading(new Pose2d(-24, -31, Math.PI / 2), Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineToSplineHeading(redStack, Math.PI)
                                .waitSeconds(waitTime)
                                .splineTo(redGate, 0)
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(redBackdrop, redBackdrop.getHeading())
                                .forward(5,
                                        SampleMecanumDrive.getVelocityConstraint(slowSpeed, Math.PI, 15),
                                        SampleMecanumDrive.getAccelerationConstraint(60))
                                .waitSeconds(waitTime)
                                .back(backdropBackDistance)
                                .splineToLinearHeading(redFrontPark, redFrontPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity redFrontCenter = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redFrontStart)
                                .splineToSplineHeading(new Pose2d(-28, -30, Math.PI / 2), Math.PI / 2)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineTo(redGate, 0)
                                .splineToSplineHeading(redBackdrop, redBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(redFrontPark, redFrontPark.getHeading())
                                .build()
                );
        RoadRunnerBotEntity redFrontRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(redFrontStart)
                                .splineToSplineHeading(new Pose2d(-38, -45, Math.PI / 4), Math.PI / 4)
                                .splineToSplineHeading(new Pose2d(-36, -35, 0), 0)
                                .waitSeconds(waitTime)
                                .back(spikeMarkBackDistance)
                                .splineTo(redGate, 0)
                                .splineToSplineHeading(redBackdrop, redBackdrop.getHeading())
                                .waitSeconds(waitTime)
                                .splineToSplineHeading(redFrontPark, redFrontPark.getHeading())
                                .build()
                );

        RoadRunnerBotEntity test1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d())
                        .splineTo(new Vector2d(30, 30), Math.PI/2)
                .build()
        );
        RoadRunnerBotEntity test2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d())
                        .splineToLinearHeading(new Pose2d(30, 30, Math.PI/2), Math.PI/2)
                .build()
        );
        RoadRunnerBotEntity test3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d())
                        .splineToSplineHeading(new Pose2d(30, 30, Math.PI/2), Math.PI/2)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueBackLeft)
//                .addEntity(blueFrontRight)
//                .addEntity(redBackLeft)
//                .addEntity(redFrontLeft)
                .addEntity(test1)
                .addEntity(test2)
                .addEntity(test3)
                .start();
    }
}