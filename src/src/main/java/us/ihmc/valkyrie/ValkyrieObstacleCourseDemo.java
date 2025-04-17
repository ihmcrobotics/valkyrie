package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.valkyrie.operatorInterface.ValkyrieOperatorUserInterface;

public class ValkyrieObstacleCourseDemo
{
   public static void main(final String[] args) throws JSAPException
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DefaultCommonAvatarEnvironment());
      simulationStarter.setRunMultiThreaded(true);

      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, ValkyrieOperatorUserInterface.class, null, DRCObstacleCourseStartingLocation.values());
   }
}
