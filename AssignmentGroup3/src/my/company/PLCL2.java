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
import com.ttsnetwork.modules.standard.Robot6DOF2;
import com.ttsnetwork.modulespack.conveyors.ConveyorBox;
import com.ttsnetwork.modulespack.conveyors.SensorCatch;

/**
 *
 * @author michel
 */
public class PLCL2 extends ProgrammableLogics {

    private ConveyorBox dBox;
    private ConveyorBox abd3Box;
    private ConveyorBox abd4Box;

    public Robot6DOF2 R4;
    public Robot6DOF2 R5;

    private IRobotCommands r4Cmd;
    private IRobotCommands r5Cmd;

    public ConveyorLine2 D;
    public ConveyorLine2 ABD3;
    public ConveyorLine2 ABD4;
    public ConveyorLine2 ABD6;
    public ConveyorLine2 E1;

    private IConveyorCommands dCmd;
    private IConveyorCommands abd3Cmd;
    private IConveyorCommands abd4Cmd;
//    private IConveyorCommands abd5Cmd;
//    private IConveyorCommands abd6Cmd;
//    private IConveyorCommands e1Cmd;

    private ISensorProvider s1dSens;
    private ISensorProvider s1abd3Sens;
    private ISensorProvider s1abd4Sens;

    private void dBoxArrived(SensorCatch sc) {
        // record the arrived box
        dBox = sc.box;
        schedule.startSerial();
        {
            // immediately stop conveyor
            dCmd.speedSet(0);
            doAbdAssembly();
        }
        schedule.end();
    }

    private void abd3BoxArrived(SensorCatch sc) {
        abd3Box = sc.box;
        schedule.startSerial();
        {
            abd3Cmd.speedSet(0);
            doAbdAssembly();
        }
        schedule.end();
    }

    private void abd4BoxArrived(SensorCatch sc) {
        abd4Box = sc.box;
        schedule.startSerial();
        {
            abd4Cmd.speedSet(0);
            doAbdWelding();
        }
        schedule.end();
    }

    private void doAbdAssembly() {
        if (dBox != null && abd3Box != null) {
            schedule.startSerial();
            {
                Transform dTargetOffset = BoxUtils.targetOffset(dBox, 0, 0, BoxUtils.zSize(dBox) * 2, 0, 0, 0);
                r4Cmd.move(dTargetOffset, 2000);
                r4Cmd.moveLinear(BoxUtils.targetTop(dBox), 2000);
                dCmd.remove(dBox);
                r4Cmd.pick(driver.getObject(dBox.entityId));
                r4Cmd.moveLinear(dTargetOffset, 2000);
                Transform abd3TargetOffset = BoxUtils.targetOffset(abd3Box, 0, 0, BoxUtils.zSize(abd3Box) + BoxUtils.zSize(dBox) * 3, 0, 0, 0);
                r4Cmd.move(abd3TargetOffset, 2000);
                r4Cmd.moveLinear(BoxUtils.targetOffset(abd3Box, 0, 0, BoxUtils.zSize(abd3Box) + BoxUtils.zSize(dBox) * 2, 0, 0, 0), 2000);
                r4Cmd.release();
                schedule.attach(dBox.entity, abd3Box.entity);
                r4Cmd.moveLinear(abd3TargetOffset, 2000);
                // reset the speed of the conveyors
                dCmd.speedSet(1);
                abd3Cmd.speedSet(1);
            }
            schedule.end();
            dBox = null;
            abd3Box = null;
        }
    }

    private void doAbdWelding() {
        schedule.startSerial();
        {
            Transform abd4TargetOffset = BoxUtils.targetOffset(abd4Box, 0, 0, BoxUtils.zSize(abd4Box) * 4, 0, 0, 180);
            Transform abd4TopTargetOffset = BoxUtils.targetOffset(abd4Box, 0, 0, BoxUtils.zSize(abd4Box) * 3, 0, 0, 180);
            r5Cmd.move(abd4TargetOffset, 2000);

            r5Cmd.moveLinear(abd4TopTargetOffset, 2000);
            r5Cmd.moveLinear(abd4TargetOffset, 2000);
            abd4Cmd.speedSet(1);
        }
        schedule.end();
        abd4Box = null;
    }

    @Override
    public void onInit() {
        r4Cmd = R4.create(module);
        r5Cmd = R5.create(module);

        dCmd = D.createCommands(module);
        s1dSens = D.createSensors(module);

        abd3Cmd = ABD3.createCommands(module);
        s1abd3Sens = ABD3.createSensors(module);

        abd4Cmd = ABD4.createCommands(module);
        s1abd4Sens = ABD4.createSensors(module);

//        e1Cmd = E1.createCommands(module);

        s1dSens.registerOnSensors(this::dBoxArrived, "S1D");
        s1abd3Sens.registerOnSensors(this::abd3BoxArrived, "S1ABD3");
        s1abd4Sens.registerOnSensors(this::abd4BoxArrived, "S1ABD4");

    }

}
