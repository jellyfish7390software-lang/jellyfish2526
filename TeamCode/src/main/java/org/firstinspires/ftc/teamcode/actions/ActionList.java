package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;

import org.firstinspires.ftc.teamcode.Robot;

public class ActionList {
    Robot bot;

    public ActionList(Robot bot) {
        this.bot = bot;
    }

    public Action setShooterVelocity(int vel) {
        return new InstantAction(() -> bot.setShooterVelocity(vel));
    }

    public Action setIntakePower(double power) {
        return new InstantAction(() -> bot.intakePower(power));
    }
    public Action startCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = true);
    }
    public Action stopCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = false);
    }
}
