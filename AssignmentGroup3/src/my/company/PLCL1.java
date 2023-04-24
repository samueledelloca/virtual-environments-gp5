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
    public ConveyorLine2 ABD1;
    public ConveyorLine2 C;
    public Robot6DOF2 R1;
    public Robot6DOF2 R2;

    private ISensorProvider s1a1Sens;
    private ISensorProvider s1a2Sens;
    private ISensorProvider s1a3Sens;
    private ISensorProvider s1bSens;
    private ISensorProvider s1cSens;

    private IConveyorCommands a1Cmd;
    private IConveyorCommands a2Cmd;
    private IConveyorCommands a3Cmd;
    private IConveyorCommands bCmd;
    private IConveyorCommands abd1Cmd;
    private IConveyorCommands cCmd;

    private ConveyorBox boxOnA;
    private ConveyorBox boxOnB;
    private ConveyorBox boxOnAB;
    private ConveyorBox boxOnC;

    private IRobotCommands r1Cmd;
    private IRobotCommands r2Cmd;

    private void boxAArrived(SensorCatch sc) {
        // record the arrived box
        boxOnA = sc.box;
        schedule.startSerial();
        {
            // immediately stop conveyor
            a1Cmd.speedSet(0);
            doABAssembly();
        }
        schedule.end();
    }

    private void boxBArrived(SensorCatch sc) {
        boxOnB = sc.box;
        schedule.startSerial();
        {
            bCmd.speedSet(0);
            doABAssembly();
        }
        schedule.end();
    }
    
    private void boxABArrived(SensorCatch sc) {
        // record the arrived box
        boxOnAB = sc.box;
        schedule.startSerial();
        {
            // immediately stop conveyor
            a3Cmd.speedSet(0);
            doABCAssembly();
        }
        schedule.end();
    }

    private void boxCArrived(SensorCatch sc) {
        boxOnC = sc.box;
        schedule.startSerial();
        {
            cCmd.speedSet(0);
            doABCAssembly();
        }
        schedule.end();
    }

    private void pushA2Box(SensorCatch sc) {
        if ("A1".equals(sc.box.entity.getProperty("rfid"))) {
            schedule.startSerial();
            {
                a2Cmd.speedSet(0);
                schedule.callFunction(this::extractP1);
                schedule.waitTime(1000);
                schedule.callFunction(this::retractP1);
                a2Cmd.speedSet(1);
            }
            schedule.end();
        }
    }

    private void doABAssembly() {
        // assembly can be performed only if both boxes are present
        if (boxOnA != null && boxOnB != null) {
            // execute the series of assembly operations
            schedule.startSerial();
            {
                r1Cmd.move(BoxUtils.targetTop(boxOnB), 2000);
                r1Cmd.pick(boxOnB.entity);
                // remove the box that is no more on the conveyor
                bCmd.remove(boxOnB);
                r1Cmd.move(BoxUtils.targetOffset(boxOnA, 0, 0, BoxUtils.zSize(boxOnA) + BoxUtils.zSize(boxOnB), 0, 0, 90), 2000);
                r1Cmd.release();
                schedule.attach(boxOnB.entity, boxOnA.entity);
                // reset the speed of the conveyors
                a1Cmd.speedSet(1);
                bCmd.speedSet(1);
            }
            schedule.end();
            // forget the operated boxes
            boxOnA = null;
            boxOnB = null;
        }
    }
    
    private void doABCAssembly() {
        // assembly can be performed only if both boxes are present
        if (boxOnAB != null && boxOnC != null) {
            // execute the series of assembly operations
            schedule.startSerial();
            {
                r2Cmd.move(BoxUtils.targetTop(boxOnC), 2000);
                r2Cmd.pick(boxOnC.entity);
                // remove the box that is no more on the conveyor
                cCmd.remove(boxOnC);
                r2Cmd.move(BoxUtils.targetOffset(boxOnAB, 0, 0, BoxUtils.zSize(boxOnAB) + BoxUtils.zSize(boxOnC), 0, 0, 90), 2000);
                r2Cmd.release();
                schedule.attach(boxOnC.entity, boxOnAB.entity);
                // reset the speed of the conveyors
                a3Cmd.speedSet(1);
                cCmd.speedSet(1);
            }
            schedule.end();
            // forget the operated boxes
            boxOnAB = null;
            boxOnC = null;
        }
    }

    @Override
    public void onInit() {
//        srcCmd = SA.createSkill(module);

        a1Cmd = A1.createCommands(module);
        a2Cmd = A2.createCommands(module);
        a3Cmd = A3.createCommands(module);
        bCmd = B.createCommands(module);
        abd1Cmd = ABD1.createCommands(module);
        cCmd = C.createCommands(module);

        s1a1Sens = A1.createSensors(module);
        s1a2Sens = A2.createSensors(module);
        s1a3Sens = A3.createSensors(module);
        s1bSens = B.createSensors(module);
        s1cSens = C.createSensors(module);
        r1Cmd = R1.create(module);
        r2Cmd = R2.create(module);

        s1a1Sens.registerOnSensors(this::boxAArrived, "S1A1");
        s1a2Sens.registerOnSensors(this::pushA2Box, "S1A2");
        s1a3Sens.registerOnSensors(this::boxABArrived, "S1A3");
        s1bSens.registerOnSensors(this::boxBArrived, "S1B");
        s1cSens.registerOnSensors(this::boxCArrived, "S1C");

        schedule.startSerial();
        {
            schedule.waitTime(1000);
//            srcCmd.create(null);
//            schedule.waitTime(4000);
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
