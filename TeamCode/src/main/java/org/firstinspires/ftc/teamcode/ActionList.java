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
}
