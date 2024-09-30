package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This class contains the arm configurations for Nadia; this allows for the Cycloid arms, the TMotor arms, and the Cycloid arms
 * with the TMotor forearms with the SAKE hands.
 */
public class ValkyriePresetArmConfigurations
{
   private static final SideDependentList<double[]> INITIAL_SETUP = new SideDependentList<>();
   private static final SideDependentList<double[]> STAND_PREP = new SideDependentList<>();
   private static final SideDependentList<double[]> HOME = new SideDependentList<>();
   private static final SideDependentList<double[]> TUCKED_UP_ARMS = new SideDependentList<>();

   // To tune these values, run NadiaModelViewer as it allows you to tune joint angles with sliders
   static
   {
      for (RobotSide side : RobotSide.values)
      {
         INITIAL_SETUP.put(side, new double[]{0.7, side.negateIfRightSide(0.2), side.negateIfRightSide(0.0), -1.4, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         STAND_PREP.put(side, new double[]{0.3, side.negateIfRightSide(0.1), side.negateIfRightSide(0.2), -0.4, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         HOME.put(side, new double[]{0.7, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), -1.2, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
         TUCKED_UP_ARMS.put(side, new double[]{0.6, side.negateIfRightSide(0.0), side.negateIfRightSide(0.25), -1.3, side.negateIfRightSide(0.0), side.negateIfRightSide(0.0), side.negateIfRightSide(0.0)});
      }
   }

   /**
    * @return a copy so the original values don't get modified.
    */
   public static double[] getPresetArmConfiguration(RobotSide side, PresetArmConfiguration presetArmConfiguration)
   {
      double[] jointAngles;
      jointAngles = new double[HOME.get(RobotSide.LEFT).length];
      getArmConfigurations(side, presetArmConfiguration, jointAngles);

      return jointAngles;
   }

   public static void getArmConfigurations(RobotSide side, PresetArmConfiguration presetArmConfiguration, double[] jointAnglesToPack)
   {
      switch (presetArmConfiguration)
      {
         case INITIAL_SETUP -> System.arraycopy(INITIAL_SETUP.get(side), 0, jointAnglesToPack, 0, INITIAL_SETUP.get(side).length);
         case STAND_PREP -> System.arraycopy(STAND_PREP.get(side), 0, jointAnglesToPack, 0, STAND_PREP.get(side).length);
         case HOME -> System.arraycopy(HOME.get(side), 0, jointAnglesToPack, 0, HOME.get(side).length);
         case TUCKED_UP_ARMS -> System.arraycopy(TUCKED_UP_ARMS.get(side), 0, jointAnglesToPack, 0, TUCKED_UP_ARMS.get(side).length);
      }
   }
}
