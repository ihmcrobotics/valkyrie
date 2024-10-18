package us.ihmc.valkyrie.parameters;

import us.ihmc.footstepPlanning.LocomotionParameters;

public class ValkyrieLocomotionParameters extends LocomotionParameters
{
   public ValkyrieLocomotionParameters()
   {
      super(ValkyrieLocomotionParameters.class);
      loadUnsafe();
   }
}
