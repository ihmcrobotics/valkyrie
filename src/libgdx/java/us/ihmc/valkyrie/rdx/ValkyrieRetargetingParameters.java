package us.ihmc.valkyrie.rdx;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieStandPrepSetpoints;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;

public class ValkyrieRetargetingParameters extends RetargetingParameters
{
   private final ValkyrieJointMap jointMap;
   private final WholeBodySetpointParameters homePoseParameters;

   public ValkyrieRetargetingParameters(ValkyrieJointMap jointMap, FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);

      this.jointMap = jointMap;
      this.homePoseParameters = new ValkyrieStandPrepSetpoints(jointMap);
   }

   @Override
   public float getHomePoint(String jointName)
   {
      return (float) homePoseParameters.getSetpoint(jointName);
   }

   @Override
   public float getArmHomePoint(RobotSide robotSide, ArmJointName jointName)
   {
      return (float) homePoseParameters.getSetpoint(jointMap.getArmJointName(robotSide, jointName));
   }

   /**
    * Pelvis height with extended legs. Not fully extended, but far enough from singularity
    */
   @Override
   public double getPelvisHeightExtendedLegs()
   {
      return 1.02;
   }

   @Override
   public double getArmLength()
   {
      return 0.73;
   }

   @Override
   public YawPitchRoll getControlFrameOrientationInBodyFrame(VRTrackedSegmentType tracker)
   {
      YawPitchRoll defaultYawPitchRoll = super.getControlFrameOrientationInBodyFrame(tracker);
      switch (tracker)
      {
         // TODO: tune up
         case LEFT_WRIST ->
         {
            return new YawPitchRoll(Math.PI / 2.0, 0.0, 0.0);
         }
         // TODO: tune up
         case RIGHT_WRIST ->
         {
            return new YawPitchRoll(-Math.PI / 2.0, 0.0, 0.0);
         }
         // TODO: tune up
         case CHEST, WAIST, LEFT_ANKLE, RIGHT_ANKLE ->
         {
            return new YawPitchRoll(-Math.PI / 2.0, 0.0, -Math.PI / 2.0);
         }
      }
      return defaultYawPitchRoll;
   }
}
