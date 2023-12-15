package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        double forwardDistance = 26;
        double waitTime = 1;

        Pose2d blueBackStart = new Pose2d(11, 62.5, -Math.PI / 2);
        Vector2d blueBackdropCoords = new Vector2d(48, 37);
        Pose2d[] bluePark = {
                new Pose2d(47, 62, Math.PI),
                new Pose2d(60, 62, Math.PI)
        };
        int multiplier = -1;
        Vector2d redBackdropCoords = new Vector2d(50, -35);

        Pose2d[] spikeMarkPoses = {
                new Pose2d(22.5, 28, -Math.PI / 2),
                new Pose2d(11, 22, Math.PI),
                new Pose2d(.5, 27, -Math.PI / 2)
        };

        RoadRunnerBotEntity blueBack = new DefaultBotBuilder(meepMeep)
                // set to either red or blue alliance
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueBackStart)
                                .splineToLinearHeading(spikeMarkPoses[1], Math.PI)
                                .waitSeconds(waitTime)
                                .back(10)
                                .turn(-Math.PI / 2)
                                .lineToLinearHeading(bluePark[1])
                                .build()
                );

        RoadRunnerBotEntity[] blueBacks = new RoadRunnerBotEntity[3];
        for(int i = 0; i < 3; i++) {
            Pose2d spikeMarkPose = spikeMarkPoses[i];
            int time = 5 * i;
            blueBacks[i] = new DefaultBotBuilder(meepMeep)
                    .setColorScheme(new ColorSchemeBlueLight())
                    .setConstraints(60, 60, Math.PI, Math.PI, 15)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(blueBackStart)
                                    .waitSeconds(time)
                                    .splineToLinearHeading(spikeMarkPose, -Math.PI / 2)
                                    .waitSeconds(waitTime)
                                    .back(10)
                                    .turn(-Math.PI / 2)
                                    .splineToLinearHeading(bluePark[1], 0)
                                    .build()
                    );
        }

        Pose2d startPose = new Pose2d(-36, 62, -Math.PI / 2);
        Pose2d blueStackCoords = new Pose2d(-58, 36, Math.PI);
        Vector2d bluePark2 = new Vector2d(60, 14);
        RoadRunnerBotEntity blueFront = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .forward(forwardDistance)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(blueStackCoords)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(new Pose2d(-24, 12, 0))
                                .lineTo(bluePark2)
                                .build()
                );

        RoadRunnerBotEntity redBack = new DefaultBotBuilder(meepMeep)
                // set to either red or blue alliance
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(90)))
                                .forward(forwardDistance)
                                .waitSeconds(waitTime)
                                .turn(-Math.PI / 2)
                                .lineTo(redBackdropCoords)
                                .waitSeconds(waitTime)
                                .strafeRight(23)
                                .lineTo(new Vector2d(60, -60))
                                .build()
                );

        Pose2d redStackCoords = new Pose2d(-58, -35, Math.PI);
        Vector2d redPark = new Vector2d(60, -11);
        RoadRunnerBotEntity redFront = new DefaultBotBuilder(meepMeep)
                // set to either red or blue alliance
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.PI / 2))
                                .forward(forwardDistance)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(redStackCoords)
                                .waitSeconds(waitTime)
                                .lineToLinearHeading(new Pose2d(-22, -11, 0))
                                .lineTo(redPark)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBack)
                //.addEntity(blueBacks[0])
                //.addEntity(blueBacks[1])
                //.addEntity(blueBacks[2])
                .start();
    }
}