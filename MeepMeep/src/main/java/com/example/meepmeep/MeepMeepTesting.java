package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(400);
        Pose2d beginPose = new Pose2d(-52, 56, 0);
        RoadRunnerBotEntity botLeft = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();
        botLeft.runAction(botLeft.getDrive().actionBuilder(beginPose)
                        .splineTo(new Vector2d(-48, 24), Math.toRadians(225))
                        .waitSeconds(0.5)
                        .splineTo(new Vector2d(-12, 38), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(-24,24), Math.toRadians(135))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(12, 38), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(-24, 24), Math.toRadians(135))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(38,38), Math.toRadians(90))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(-24,24), Math.toRadians(135))



                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botLeft)
                .start();


        // Mirrored bot (right side, x flipped)
        Pose2d beginPoseRight = new Pose2d(52, 56, Math.toRadians(180));
        RoadRunnerBotEntity botRight = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .build();
        botRight.runAction(botRight.getDrive().actionBuilder(beginPoseRight)
                .splineTo(new Vector2d(48, 24), Math.toRadians(-45))   // mirror of (−48,24) at 225°
                .waitSeconds(0.5)
                .splineTo(new Vector2d(12, 38), Math.toRadians(90))    // mirror of (−12,38)
                .waitSeconds(1)
                .splineTo(new Vector2d(24, 24), Math.toRadians(45))    // mirror of (−24,24)
                .waitSeconds(1)
                .splineTo(new Vector2d(-12, 38), Math.toRadians(90))   // mirror of (12,38)
                .waitSeconds(1)
                .splineTo(new Vector2d(24, 24), Math.toRadians(45))    // mirror of (−24,24)
                .waitSeconds(1)
                .splineTo(new Vector2d(-38, 38), Math.toRadians(90))   // mirror of (38,38)
                .waitSeconds(1)
                .splineTo(new Vector2d(24, 24), Math.toRadians(45))    // mirror of (−24,24)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botLeft)
                .addEntity(botRight)
                .start();
    }
}