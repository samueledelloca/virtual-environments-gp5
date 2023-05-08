/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package my.company;

import com.ttsnetwork.commands.Transform;
import com.ttsnetwork.core.AWJoint;
import com.ttsnetwork.core.AWRS;
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
 * @author samu
 */
public class PLCL2 extends ProgrammableLogics {

    public ConveyorBox dBox;
    public ConveyorBox abd3Box;

    public Robot6DOF2 R4;
    public IRobotCommands r4Cmd;

    public ConveyorLine2 D;
    public ConveyorLine2 ABD3;

    public IConveyorCommands dCmd;
    public IConveyorCommands abd3Cmd;

    public ISensorProvider dSen;
    public ISensorProvider abd3Sen;

    private void dBoxArrived(SensorCatch sc) {
        // record the arrived box
        dBox = sc.box;
        schedule.startSerial();
        {
            // immediately stop conveyor
            dCmd.speedSet(0);
            doAssembly();
        }
        schedule.end();
    }

    private void abd3BoxArrived(SensorCatch sc) {
        abd3Box = sc.box;
        schedule.startSerial();
        {
            abd3Cmd.speedSet(0);
            doAssembly();
        }
        schedule.end();
    }

    private void doAssembly() {
        if (dBox != null && abd3Box != null) {
            schedule.startSerial();
            {
//                AWRS entity = abd3Box.entity;
//                while(entity.getChildren() == null) {
//                    entity = entity.getChildren().get(0).getChild();
//                }
//                System.out.println(entity.getFrame(null));
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

    @Override
    public void onInit() {
        r4Cmd = R4.create(module);

        dCmd = D.createCommands(module);
        dSen = D.createSensors(module);

        abd3Cmd = ABD3.createCommands(module);
        abd3Sen = ABD3.createSensors(module);

        dSen.registerOnSensors(this::dBoxArrived, "S1D");
        abd3Sen.registerOnSensors(this::abd3BoxArrived, "S1ABD3");
    }

}
