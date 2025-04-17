package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.environments.DRCFinalsStartingLocation;
import us.ihmc.avatar.environments.DRCFinalsEnvironment;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.valkyrie.operatorInterface.ValkyrieOperatorUserInterface;

public class ValkyrieDRCFinalsDemo
{
   public static void main(final String[] args) throws JSAPException
   {
      boolean door = true;
      boolean drill = true;
      boolean valve = true;
      boolean walking = true;
      boolean stairs = true;
      
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DRCFinalsEnvironment(door, drill, valve, walking, stairs));
      simulationStarter.setRunMultiThreaded(true);

      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, ValkyrieOperatorUserInterface.class, null, DRCFinalsStartingLocation.values());
   }
}
