package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Robot;

public class ActionList {
    Robot bot;

    public ActionList(Robot bot) {
        this.bot = bot;
    }

    public Action setShooterVelocity(int vel) {
        return new InstantAction(() -> bot.setShooterVelocity(vel));
    }
    public SequentialAction combine(Action... actions) {
        return new SequentialAction(actions);
    }

    public Action setHood(double pos) {
        return new InstantAction(() -> bot.hood.setPosition(pos));
    }
    public Action setIntakePower(double power) {
        return new InstantAction(() -> bot.intakePower(power));
    }
    public Action setTransferPower(double power) {
        return new InstantAction(() -> bot.transferPower(power));
    }
    public Action stopDt() {
        return new InstantAction(() -> bot.drive.setPowers());
    }
    public Action setTurretPos(int pos) {
        return new InstantAction(() -> bot.setTurretPos(pos));
    }
    public Action openHardStop() {
        return new InstantAction(() -> bot.hardstop.setPosition(Robot.HardstopOpen));
    }
    public Action closeHardStop() {
        return new InstantAction(() -> bot.hardstop.setPosition(Robot.HardstopClose));
    }
    public Action startCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = true);
    }
    public Action stopCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = false);
    }
}
