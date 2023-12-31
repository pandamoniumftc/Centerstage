package org.firstinspires.ftc.teamcode.CurrentSeason.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractTeleOp;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.BaoBao;

@TeleOp (name="main tele op")
public class MainTeleAwp extends AbstractTeleOp {
    BaoBao robot;

    @Override
    public AbstractRobot instantiateRobot() {
        robot = new BaoBao(this);

        return robot;
    }
}
