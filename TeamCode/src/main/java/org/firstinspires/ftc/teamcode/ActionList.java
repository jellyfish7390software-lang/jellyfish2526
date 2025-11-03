package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;

public class ActionList {
    Robot bot;

    public ActionList(Robot bot) {
        this.bot = bot;
    }

    public Action setShooterVelocity(int vel) {
        return new InstantAction(() -> bot.setShooterVelocity(vel));
    }
    public Action setTransferPosition(int pos) {
        return new InstantAction(() -> bot.setTransferPosition(pos));
    }
    public Action incrementTransfer(int increment) {
        return new InstantAction(() -> bot.incrementTransfer(increment));
    }
    public Action setIntakePower(double power) {
        return new InstantAction(() -> bot.intakePower(power));
    }
    public Action turnTransfer() {
        return new InstantAction(() -> bot.incrementTransfer(8192/3));
    }
    public Action startCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = true);
    }
    public Action stopCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = false);
    }
}
