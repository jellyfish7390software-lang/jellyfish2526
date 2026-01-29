
package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepMeep {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(400);

        // TOP bot (Blue Side))
        Pose2d beginPoseTop = new Pose2d(-58, 39.5, Math.toRadians(0));
        RoadRunnerBotEntity botTop = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(90), 16)
                .build();
        botTop.runAction(botTop.getDrive().actionBuilder(beginPoseTop)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(-27, 22), Math.toRadians(40))
                        .waitSeconds(0.75)
                        .setTangent(Math.toRadians(-50))
                        .splineToLinearHeading(new Pose2d(16, 58, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                        .waitSeconds(0.75)
                        .setTangent(Math.toRadians(-50))
                        .splineToLinearHeading(new Pose2d(10, 66, Math.toRadians(135)), Math.toRadians(135))
                        .waitSeconds(3)
                        .setTangent(Math.toRadians(290))
                        .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                        .waitSeconds(0.75)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(40, 58, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))
                        .waitSeconds(0.75)
                        .setTangent(Math.toRadians(-30))
                        .splineToLinearHeading(new Pose2d(-12, 58, Math.toRadians(90)), Math.toRadians(90))
                        .waitSeconds(0.5)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-27, 22, Math.toRadians(40)), Math.toRadians(160))


                .build());

        // BOTTOM bot (Red Side)
        Pose2d beginPoseBottom = new Pose2d(-56, -56, 144);
        RoadRunnerBotEntity botBottom = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .setColorScheme(new ColorSchemeRedDark()) // just to distinguish
                .build();
        botBottom.runAction(botBottom.getDrive().actionBuilder(beginPoseBottom)
                .setReversed(false)
                .strafeTo(new Vector2d(-31, -31))   // was 225°
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, -38, Math.toRadians(-90)), Math.toRadians(-90))    // was 90°
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, -24, Math.toRadians(-135)), Math.toRadians(135))  // was 135°
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(12, -38, Math.toRadians(-90)), Math.toRadians(-90))     // was 90°
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, -24, Math.toRadians(-135)), Math.toRadians(-135))  // was 135°
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38, -38, Math.toRadians(-90)), Math.toRadians(-90))     // was 90°
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, -24, Math.toRadians(-135)), Math.toRadians(-135))  // was 135°
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botTop)
                .setAxesInterval(72)
                .removeEntity(MeepMeep.getDEFAULT_AXES_ENTITY())
                .start();
    }
}