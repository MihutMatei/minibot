package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "TELEMETRY")
public class telemetry extends LinearOpMode {
    int i=1,j;
    int start1(){
        telemetry.addData("Do you want to start autonomous period?",  " ");
        telemetry.addLine();
        telemetry.addData("YES",  " PRESS DPAD_DOWN");
        telemetry.addLine();
        telemetry.addData("NO",  "  PRESS DPAD_UP");
        telemetry.update();


        int v=0;
        while(i!=2)
        {
            if(gamepad1.dpad_down)i=2;
            if(gamepad1.dpad_up)
            {
                i=2;
                v=1;
            }
        }
        if(v==1)i=1;
        return i;




    }
    void wfs(){
        telemetry.update();
        telemetry.addData("Waiting for start","Indeed I am");
        telemetry.update();
        waitForStart();
    }
    int  BoR()
    {   telemetry.update();
        telemetry.addData("Blue alliance or Red alliance?"," ");
        telemetry.addLine();
        telemetry.addData("BLUE "," Press DPAD_LEFT");
        telemetry.addLine();
        telemetry.addData("RED"," Press DPAD_RIGHT");
        telemetry.update();
        j=2;
        // sleep(5000);
        while(j<3)
        {
            if(gamepad1.dpad_left)j=3;
            if(gamepad1.dpad_right)j=4;
        }
        return j;

    }
    void red()
    {
        telemetry.update();
        telemetry.addData("Red alliance AUTONOMUS","  ");
        telemetry.addLine();
        telemetry.addData("FULL AUTONOMUS"," Press X");
        telemetry.addLine();
        telemetry.addData("PARKING"," Press A");
        telemetry.addLine();
        telemetry.addData("FULL AUTO+PARKING"," Press O");
        telemetry.update();
    }

    void blue()
    {
        telemetry.update();
        telemetry.addData("Blue alliance AUTONOMUS"," ");
        telemetry.addLine();
        telemetry.addData("FULL AUTONOMUS"," Press X");
        telemetry.addLine();
        telemetry.addData("PARKING"," Press A");
        telemetry.addLine();
        telemetry.addData("FULL AUTO+PARKING"," Press O");
        telemetry.update();
        telemetry.update();

    }


    @Override
    public void runOpMode() throws InterruptedException {
        start1();
        if(i==2){
            BoR();
        }
        if(i==1)
            wfs();
        i=BoR();
        if(i==3)blue();
        if(i==4)red();

        waitForStart();

        while(opModeIsActive())
        {

        }
    }

}
