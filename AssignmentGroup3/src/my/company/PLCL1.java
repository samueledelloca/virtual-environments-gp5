/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package my.company;

import com.ttsnetwork.commands.Transform;
import com.ttsnetwork.modules.standard.BoxUtils;
import com.ttsnetwork.modules.standard.ConveyorLine2;
import com.ttsnetwork.modules.standard.IConveyorCommands;
import com.ttsnetwork.modules.standard.IRobotCommands;
import com.ttsnetwork.modules.standard.ISensorProvider;
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
    public Pusher P1;

    public ConveyorLine2 A1;
    public ConveyorLine2 A2;
    public ConveyorLine2 A3;
    public ConveyorLine2 B;
    public ConveyorLine2 ABD1;
    public ConveyorLine2 C;
    public ConveyorLine2 ABC;

    public Robot6DOF2 R1;
    public Robot6DOF2 R2;
    public Robot6DOF2 R3;

    private ISensorProvider s1a1Sens;
    private ISensorProvider s1a2Sens;
    private ISensorProvider s1a3Sens;
    private ISensorProvider s1bSens;
    private ISensorProvider s1cSens;
    private ISensorProvider s1abcSens;

    private IConveyorCommands a1Cmd;
    private IConveyorCommands a2Cmd;
    private IConveyorCommands a3Cmd;
    private IConveyorCommands bCmd;
    private IConveyorCommands abd1Cmd;
    private IConveyorCommands cCmd;
    public IConveyorCommands abcCmd;

    private ConveyorBox boxOnA;
    private ConveyorBox boxOnB;
    private ConveyorBox boxOnAB;
    private ConveyorBox boxOnC;
    private ConveyorBox boxOnABC;

    private IRobotCommands r1Cmd;
    private IRobotCommands r2Cmd;
    private IRobotCommands r3Cmd;

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
    
    private void boxABCArrived(SensorCatch sc) {
        boxOnABC = sc.box;
        schedule.startSerial();
        {
            abcCmd.speedSet(0);
            doABCWelding();
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
                Transform bTargetOffset = BoxUtils.targetOffset(boxOnB, 0, 0, BoxUtils.zSize(boxOnB) * 2, 0, 0, 0);
                r1Cmd.move(bTargetOffset, 2000);
                r1Cmd.moveLinear(BoxUtils.targetTop(boxOnB), 2000);
                bCmd.remove(boxOnB);
                r1Cmd.pick(driver.getObject(boxOnB.entityId));
                r1Cmd.moveLinear(bTargetOffset, 2000);
                Transform aTargetOffset = BoxUtils.targetOffset(boxOnA, 0, 0, BoxUtils.zSize(boxOnA) + BoxUtils.zSize(boxOnB) * 2, 0, 0, -90);
                r1Cmd.move(aTargetOffset, 2000);
                r1Cmd.moveLinear(BoxUtils.targetOffset(boxOnA, 0, 0, BoxUtils.zSize(boxOnA) + BoxUtils.zSize(boxOnB), 0, 0, -90), 2000);
                r1Cmd.release();
                schedule.attach(boxOnB.entity, boxOnA.entity);
                r1Cmd.moveLinear(aTargetOffset, 2000);
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
                Transform cTargetOffset = BoxUtils.targetOffset(boxOnC, 0, 0, BoxUtils.zSize(boxOnC) * 2, 0, 0, 0);
                r2Cmd.move(cTargetOffset, 2000);
                r2Cmd.moveLinear(BoxUtils.targetTop(boxOnC), 2000);
                cCmd.remove(boxOnC);
                r2Cmd.pick(driver.getObject(boxOnC.entityId));
                r2Cmd.moveLinear(cTargetOffset, 2000);
                Transform abTargetOffset = BoxUtils.targetOffset(boxOnAB, 0, 0, BoxUtils.zSize(boxOnAB) + BoxUtils.zSize(boxOnC) * 3, 0, 0, 90);
                r2Cmd.move(abTargetOffset, 2000);
                r2Cmd.moveLinear(BoxUtils.targetOffset(boxOnAB, 0, 0, BoxUtils.zSize(boxOnAB) + BoxUtils.zSize(boxOnC) * 2, 0, 0, 90), 2000);
                r2Cmd.release();
                schedule.attach(boxOnC.entity, boxOnAB.entity);
                r2Cmd.moveLinear(abTargetOffset, 2000);
                r2Cmd.move(cTargetOffset, 2000);

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
    
    private void doABCWelding() {
        schedule.startSerial();
        {
            long duration = 500;
            double  speed = 2000;
            
            Transform abcTopCenter = BoxUtils.targetOffset(boxOnABC, 0, 0, BoxUtils.zSize(boxOnABC) * 4, 0, 0, 90);
            Transform abcUpperTopLeftCorner = BoxUtils.targetOffset(boxOnABC, -100, -100, BoxUtils.zSize(boxOnABC) * 4, 45, 0, 90);
            Transform abcUpperBottomLeftCorner = BoxUtils.targetOffset(boxOnABC, -50, -50, BoxUtils.zSize(boxOnABC) * 2, 45, 0, 90);
            Transform abcLowerBottomLeftCorner = BoxUtils.targetOffset(boxOnABC, 50, -50, BoxUtils.zSize(boxOnABC) * 2, 45, 0, 90);
            Transform abcLowerBottomLeftCornerRotation = BoxUtils.targetOffset(boxOnABC, 50, -50, BoxUtils.zSize(boxOnABC) * 2, 0, 45, 90);
            Transform abcLowerBottomRightCorner = BoxUtils.targetOffset(boxOnABC, 50, 50, BoxUtils.zSize(boxOnABC) * 2, 0, 45, 90);
            Transform abcLowerBottomRightCornerRotation = BoxUtils.targetOffset(boxOnABC, 50, 50, BoxUtils.zSize(boxOnABC) * 2, -45, 0, 90);
            Transform abcUpperBottomRightCorner = BoxUtils.targetOffset(boxOnABC, -50, 50, BoxUtils.zSize(boxOnABC) * 2, -45, 0, 90);
            Transform abcUpperBottomRightCornerRotation = BoxUtils.targetOffset(boxOnABC, -50, 50, BoxUtils.zSize(boxOnABC) * 2, 0, -45, 90);
            Transform abcUpperBottomLeftCornerRotated = BoxUtils.targetOffset(boxOnABC, -50, -50, BoxUtils.zSize(boxOnABC) * 2, 0, -45, 90);
            Transform abcUpperTopLeftCornerRotated = BoxUtils.targetOffset(boxOnABC, -100, -100, BoxUtils.zSize(boxOnABC) * 4, 0, -45, 90);

            r3Cmd.move(abcTopCenter, duration);
            r3Cmd.move(abcUpperTopLeftCorner, duration);
            r3Cmd.moveLinear(abcUpperBottomLeftCorner, speed);
            r3Cmd.moveLinear(abcLowerBottomLeftCorner, speed);
            r3Cmd.moveLinear(abcLowerBottomLeftCornerRotation, speed);
            r3Cmd.moveLinear(abcLowerBottomRightCorner, speed);
            r3Cmd.moveLinear(abcLowerBottomRightCornerRotation, speed);
            r3Cmd.moveLinear(abcUpperBottomRightCorner, speed);
            r3Cmd.moveLinear(abcUpperBottomRightCornerRotation, speed);
            r3Cmd.moveLinear(abcUpperBottomLeftCornerRotated, speed);
            r3Cmd.moveLinear(abcUpperTopLeftCornerRotated, speed);
            r3Cmd.move(abcTopCenter, duration);
            abcCmd.speedSet(1);
        }
        schedule.end();
        boxOnABC = null;
    }

    @Override
    public void onInit() {
        a1Cmd = A1.createCommands(module);
        a2Cmd = A2.createCommands(module);
        a3Cmd = A3.createCommands(module);
        bCmd = B.createCommands(module);
        abd1Cmd = ABD1.createCommands(module);
        cCmd = C.createCommands(module);
        abcCmd = ABC.createCommands(module);

        s1a1Sens = A1.createSensors(module);
        s1a2Sens = A2.createSensors(module);
        s1a3Sens = A3.createSensors(module);
        s1bSens = B.createSensors(module);
        s1cSens = C.createSensors(module);
        s1abcSens = ABC.createSensors(module);
        r1Cmd = R1.create(module);
        r2Cmd = R2.create(module);
        r3Cmd = R3.create(module);

        s1a1Sens.registerOnSensors(this::boxAArrived, "S1A1");
        s1a2Sens.registerOnSensors(this::pushA2Box, "S1A2");
        s1a3Sens.registerOnSensors(this::boxABArrived, "S1A3");
        s1bSens.registerOnSensors(this::boxBArrived, "S1B");
        s1cSens.registerOnSensors(this::boxCArrived, "S1C");
        s1abcSens.registerOnSensors(this::boxABCArrived, "S1ABC");

        schedule.startSerial();
        {
            schedule.waitTime(1000);
        }
        schedule.end();
    }

    public void extractP1() {
        P1.cmdStable.write(true);
    }

    public void retractP1() {
        P1.cmdStable.write(false);
    }
}
