package us.ihmc.valkyrieRosControl;

import us.ihmc.tools.TimestampProvider;

public class SettableTimestampProvider implements TimestampProvider
{
   private long timestamp;

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   @Override
   public long getTimestamp()
   {
      return timestamp;
   }
}