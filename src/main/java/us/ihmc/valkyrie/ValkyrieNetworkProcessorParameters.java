package us.ihmc.valkyrie;

public enum ValkyrieNetworkProcessorParameters
{
   start_footstep_planning("Whether to start the footstep planner.", true),
   start_walking_preview("Whether to start the walking preview toolbox.", true),
   start_force_estimation("Whether to start the force estimation toolbox", true),
   start_kinematics_streaming_toolbox("Whether to start the kinematics streaming toolbox", true),
   start_directional_nav("Whether to start the directional navigation toolbox", true);

   private final String description;
   private final boolean defaultValue;

   private ValkyrieNetworkProcessorParameters(String description, boolean defaultValue)
   {
      this.description = description;
      this.defaultValue = defaultValue;
   }

   public String getDescription()
   {
      return description;
   }

   public boolean getDefaultValue()
   {
      return defaultValue;
   }
}
