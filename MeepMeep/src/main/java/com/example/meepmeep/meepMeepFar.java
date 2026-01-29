
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
        Pose2d beginPoseTop = new Pose2d(72 - 8.0625, 24 - 8.75, Math.toRadians(90));
        RoadRunnerBotEntity botTop = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 60, Math.toRadians(180), Math.toRadians(90), 16)
                .build();
        botTop.runAction(botTop.getDrive().actionBuilder(beginPoseTop)
                        .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(36, 24, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, 64), Math.toRadians(90))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(56, 10))
                .waitSeconds(1)
                .setTangent(Math.toRadians(110))
                .splineTo(new Vector2d(50, 64), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(110))
                .splineTo(new Vector2d(50, 64), Math.toRadians(180))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(90))

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
                .removeEntity(MeepMeep.getDEFAULT_AXES_ENTITY())
                .start();
    }
}