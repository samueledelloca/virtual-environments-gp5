/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package my.company;

import com.ttsnetwork.modules.standard.BoxUtils;
import com.ttsnetwork.modules.standard.ConveyorLine2;
import com.ttsnetwork.modules.standard.IConveyorCommands;
import com.ttsnetwork.modules.standard.IRobotCommands;
import com.ttsnetwork.modules.standard.ISensorProvider;
import com.ttsnetwork.modules.standard.ISource;
import com.ttsnetwork.modules.standard.ParametricSource2;
import com.ttsnetwork.modules.standard.ProgrammableLogics;
import com.ttsnetwork.modules.standard.Pusher;
import com.ttsnetwork.modules.standard.Robot6DOF2;
import com.ttsnetwork.modulespack.conveyors.ConveyorBox;
import com.ttsnetwork.modulespack.conveyors.SensorCatch;

/**
 *
 * @author samu
 */
public class PLCL1 extends ProgrammableLogics {

//    public ParametricSource2 SA;
//    ISource srcCmd;

    public Pusher P1;
    
    public ConveyorLine2 A1;
    public ConveyorLine2 A2;
    public ConveyorLine2 A3;
    public ConveyorLine2 B;
    public Robot6DOF2 R1;
    

    private ISensorProvider s1a1Sens;
    private ISensorProvider s1bSens;

    private IConveyorCommands a1Cmd;
    private IConveyorCommands a2Cmd;
    private IConveyorCommands a3Cmd;
    private IConveyorCommands bCmd;

    private ConveyorBox boxOn1;
    private ConveyorBox boxOn2;
    
    private IRobotCommands r1Cmd;

    private void box1Arrived(SensorCatch sc) {
        // record the arrived box
        boxOn1 = sc.box;
        schedule.startSerial();
        {
            // immediately stop conveyor
            a1Cmd.speedSet(0);
            doAssembly();
        }
        schedule.end();
    }
    
    private void box2Arrived(SensorCatch sc) {
        boxOn2 = sc.box;
        schedule.startSerial();
        {
            bCmd.speedSet(0);
            doAssembly();
        }
        schedule.end();
    }
    
    private void doAssembly(){
        // assembly can be performed only if both boxes are present
        if(boxOn1 != null && boxOn2 != null){
            // execute the series of assembly operations
            schedule.startSerial();
            {
                r1Cmd.move(BoxUtils.targetTop(boxOn2), 2000);
                r1Cmd.pick(boxOn2.entity);
                // remove the box that is no more on the conveyor
                bCmd.remove(boxOn2);
                r1Cmd.move(BoxUtils.targetOffset(boxOn1, 0, 0, BoxUtils.zSize(boxOn1) + BoxUtils.zSize(boxOn2), 0, 0, 90), 2000);
                r1Cmd.release();
                schedule.attach(boxOn2.entity, boxOn1.entity);
                // reset the speed of the conveyors
                a1Cmd.speedSet(1);
                bCmd.speedSet(1);
            }
            schedule.end();
            // forget the operated boxes
            boxOn1 = null;
            boxOn2 = null;
        }
    }

    @Override
    public void onInit() {
//        srcCmd = SA.createSkill(module);
        
        a1Cmd = A1.createCommands(module);
        a2Cmd = A2.createCommands(module);
        a3Cmd = A3.createCommands(module);
        bCmd = B.createCommands(module);
        
        s1a1Sens = A1.createSensors(module);
        s1bSens = B.createSensors(module);
        r1Cmd = R1.create(module);
        
        s1a1Sens.registerOnSensors(this::box1Arrived, "S1A1");
        s1bSens.registerOnSensors(this::box2Arrived, "S1B");

        schedule.startSerial();
        {
            schedule.waitTime(1000);
//            srcCmd.create(null);
//            schedule.waitTime(4000);
            schedule.callFunction(this::extractP1);
            schedule.waitTime(1000);
            schedule.callFunction(this::retractP1);
        }
        schedule.end();
    }

    public void extractP1() {
        P1.cmdStable.write(true);
    }

    public void retractP1() {
        P1.cmdStable.write(false);
    }

//    schedule.startSerial ();
//
//    {
//        c1Cmd.speedSet(0);
//        schedule.callFunction(this::extract);
//        schedule.waitTime(1000);
//        schedule.callFunction(this::retract);
//        c1Cmd.speedSet(1);
//    }
//
//    schedule.end ();
}
