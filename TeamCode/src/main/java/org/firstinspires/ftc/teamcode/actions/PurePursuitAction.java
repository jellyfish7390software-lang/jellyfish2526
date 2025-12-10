package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.purepursuit.math.Bezier;
import org.firstinspires.ftc.teamcode.purepursuit.math.BezierPath;
import org.firstinspires.ftc.teamcode.purepursuit.math.Path;
import org.firstinspires.ftc.teamcode.purepursuit.math.PurePursuit;

public class PurePursuitAction implements Action {
    PurePursuit purePursuit;
    Path path;
    public PurePursuitAction(PurePursuit purePursuit, Path path) {
        this.purePursuit = purePursuit;
        this.path = path;
        purePursuit.lastT = 0.0;
        purePursuit.hasReachedDestination = false;
        purePursuit.atEnd = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (path instanceof Bezier) {
            purePursuit.followPathSingle((Bezier) path, telemetryPacket);
        } else if (path instanceof BezierPath) {
            purePursuit.followPathSingle((BezierPath) path, telemetryPacket);
        } else {
            throw new IllegalArgumentException("PurePursuitAction only accepts Bezier or BezierPath!");
        }

        return !purePursuit.hasReachedDestination && purePursuit.notWithinTolerance(purePursuit.pose, purePursuit.targetPose, PurePursuit.Defaults.INSTANCE.getPosTolerance(), PurePursuit.Defaults.INSTANCE.getHTolerance());
    }
}
