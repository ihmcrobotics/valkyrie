package us.ihmc.valkyrie.perception;

import us.ihmc.log.LogTools;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.SL_BodyTrackingParameters;
import us.ihmc.zed.global.zed;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import javax.annotation.Nullable;

public class ValkyrieHardwareAutonomyProcess
{
   private static final boolean ZED_SDK_LOADED = ZEDJavaAPINativeLibrary.load();
   private static final int CAMERA_ID = 0;

   @Nullable
   private ZEDImageSensor zedSensor;
   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   private ValkyrieHardwareAutonomyProcess()
   {
      if (ZED_SDK_LOADED)
      {
         SL_BodyTrackingParameters bodyTrackingParameters = zed.sl_get_body_tracking_parameters(CAMERA_ID);
         zed.sl_enable_body_tracking(CAMERA_ID, bodyTrackingParameters);

         zedSensor = new ZEDImageSensor(CAMERA_ID, ZEDModelData.ZED_MINI, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_PERFORMANCE);
      }
      else
         LogTools.error("ZED SDK not found. Not using ZED sensor.");

      Runtime.getRuntime().addShutdownHook(new Thread(this::close, getClass().getSimpleName() + "Closer"));

      perceptionAutonomyProcess = new ValkyrieAutonomyProcess(zedSensor);
   }

   private void close()
   {
      perceptionAutonomyProcess.close();
   }

   public static void main(String[] args)
   {
      new ValkyrieHardwareAutonomyProcess();
   }
}
