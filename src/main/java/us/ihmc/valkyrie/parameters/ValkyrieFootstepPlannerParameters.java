package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.tools.property.StoredPropertySet;

public class ValkyrieFootstepPlannerParameters extends StoredPropertySet implements DefaultFootstepPlannerParametersBasics
{
   public ValkyrieFootstepPlannerParameters()
   {
      this("");
   }

   public ValkyrieFootstepPlannerParameters(String versionSuffix)
   {
      super(DefaultFootstepPlannerParameters.keys, ValkyrieFootstepPlannerParameters.class, versionSuffix);

      setCheckForBodyBoxCollisions(false);
      setIdealFootstepWidth(0.2);
      setIdealFootstepLength(0.2);
      setMaxStepReach(0.4);
      setMaxStepYaw(0.6);
      setMinStepYaw(-0.15);
      setMinStepWidth(0.2);
      setMaxStepWidth(0.4);
      setMaxStepZ(0.15);
      setBodyBoxBaseX(0.03);
      setBodyBoxBaseY(0.2);
      setBodyBoxBaseZ(0.3);
      setBodyBoxWidth(0.85);
      setBodyBoxDepth(0.4);
      setMinClearanceFromStance(0.05);
      // setCliffBaseHeightToAvoid(0.07);
      setMinDistanceFromCliffBottoms(0.04);
      setWiggleInsideDeltaTarget(0.03);
      setMaxXYWiggleDistance(0.04);
      setMaxYawWiggle(0.3);
      setAStarHeuristicsWeight(5.0);
      setYawWeight(0.15);
      setForwardWeight(2.5);

      loadUnsafe();
   }

   public static void main(String[] args)
   {
      ValkyrieFootstepPlannerParameters parameters = new ValkyrieFootstepPlannerParameters();
      parameters.save();
   }
}
