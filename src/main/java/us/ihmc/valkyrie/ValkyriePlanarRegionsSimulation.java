package us.ihmc.valkyrie;

import us.ihmc.avatar.AvatarPlanarRegionsSimulation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class ValkyriePlanarRegionsSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20190219_182005_CompareStepBeforeGap;
   private static final boolean GENERATE_GROUND_PLANE = false;
   private static final boolean ADD_EXTRA_CONTACT_POINTS = true;

   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

      if (ADD_EXTRA_CONTACT_POINTS)
      {
         FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
         robotModel.setSimulationContactPoints(simulationContactPoints);
      }

      new AvatarPlanarRegionsSimulation(robotModel, DATA_SET_TO_USE, GENERATE_GROUND_PLANE);
   }
}
