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
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(DefaultFootstepPlannerParameters.keys, ValkyrieFootstepPlannerParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
