package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class HelloWorld extends OpMode {

    @Override
    public void init(){
        telemetry.addData( "Hello", "World" );
    }

    @Override
    public void loop(){}
}
