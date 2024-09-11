package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dumbMap;

@Disabled
public class ActionRunnerCenterStageAuton {
    public LinearOpMode opMode;
    dumbMap jayBot;
    Telemetry telemetry;

    public ActionRunnerCenterStageAuton(LinearOpMode opMode, dumbMap jayBot) {
        this.opMode = opMode;
        this.jayBot = jayBot;
        telemetry = new MultipleTelemetry(this.opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

    }
/*
    public void runActions(String action) {
        telemetry.addLine("Run action");
        telemetry.addLine("action is " + action);
        switch (action) {
            case("init"):
                (jayBot.init());
                break;


        }
        telemetry.update();

    }

 */


}
