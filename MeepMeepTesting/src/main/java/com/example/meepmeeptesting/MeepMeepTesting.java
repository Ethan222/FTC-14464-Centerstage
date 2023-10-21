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

        double stackX = -62.6, stackY = -6.4;
        double junctionX = -25.9, junctionY = -5.6;
        double intakeWaitTime = .5;
        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                // set to either red or blue alliance
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-31, -61, Math.toRadians(90)))
                                // move to high junction 1st time
                                .lineTo(new Vector2d(-15, -57))
                                .lineTo(new Vector2d(-15, -12))
                                .lineTo(new Vector2d(junctionX+.3, junctionY + 1.9+.3))
                                // drop 1st cone
                                .waitSeconds(intakeWaitTime)

                                // 2nd cone - move to stack 1st time
                                .back(2.8)
                                .turn(Math.toRadians(90))
                                .lineTo(new Vector2d(stackX, stackY))
                                // pick up 2nd cone
                                .waitSeconds(intakeWaitTime)
                                // move to high junction 2nd time
                                .lineToLinearHeading(new Pose2d(junctionX - 1.3, junctionY-1.1-.3+.4, Math.toRadians(90)))
                                // drop 2nd cone
                                .waitSeconds(intakeWaitTime)
                                .lineTo(new Vector2d(-37.2-1, -7.3))
                                .turn(Math.toRadians(180))
                                .build()
                );

        double finalJunctionX = 26.1-.4, finalJunctionY = -6.4+.2;
        double finalStackX = 61.4, finalStackY = .3;
        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                // set to either red or blue alliance
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(37, -61, Math.toRadians(90)))
                                // move to high junction 1st time
                                .lineTo(new Vector2d(13, -57))
                                .lineTo(new Vector2d(15-1, -12))
                                .lineTo(new Vector2d(finalJunctionX -.5-.7, finalJunctionY + 2))
                                // drop 1st cone
                                .waitSeconds(intakeWaitTime)

                                // 2nd cone - move to stack 1st time
                                .back(2.8)
                                .turn(Math.toRadians(-90))
                                .lineTo(new Vector2d(finalStackX, finalStackY))
                                // pick up 2nd cone
                                .waitSeconds(intakeWaitTime)
                                // move to high junction 2nd time
                                .lineToLinearHeading(new Pose2d(finalJunctionX + 1, finalJunctionY -.9+.9+.4, Math.toRadians(90)))
                                // drop 2nd cone
                                .waitSeconds(intakeWaitTime)
                                .lineTo(new Vector2d(31-.4-.4, -7))
                                .turn(Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .start();
    }
}