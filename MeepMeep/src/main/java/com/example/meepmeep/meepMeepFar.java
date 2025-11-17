
package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepMeepFar {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(400);

        // TOP bot (Blue Side))
        Pose2d beginPoseTop = new Pose2d(60, 24, Math.toRadians(144));
        RoadRunnerBotEntity botTop = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 16)
                .build();
        botTop.runAction(botTop.getDrive().actionBuilder(beginPoseTop)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(153))
                .waitSeconds(0.5)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(38, 38), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(153))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(153))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(153))
                .build());

        // BOTTOM bot (Red Side)
        Pose2d beginPoseBottom = new Pose2d(60, -24, 216);
        RoadRunnerBotEntity botBottom = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 16)
                .build();
        botBottom.runAction(botBottom.getDrive().actionBuilder(beginPoseBottom)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(197))
                .waitSeconds(0.5)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(38, -38), Math.toRadians(270))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(197))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(60, -60), Math.toRadians(90))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(197))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(60, 60), Math.toRadians(270))
                .waitSeconds(1)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56, 0), Math.toRadians(197))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botTop)
                .addEntity(botBottom)
                .start();
    }
}