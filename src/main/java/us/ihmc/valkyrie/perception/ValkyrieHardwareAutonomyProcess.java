package us.ihmc.valkyrie.perception;

import us.ihmc.log.LogTools;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import javax.annotation.Nullable;

public class ValkyrieHardwareAutonomyProcess
{
   private static final boolean ZED_SDK_LOADED = ZEDJavaAPINativeLibrary.load();

   @Nullable
   private ZEDImageSensor zedSensor;
   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   private ValkyrieHardwareAutonomyProcess()
   {
      if (ZED_SDK_LOADED)
         zedSensor = new ZEDImageSensor(0, ZEDModelData.ZED_MINI, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_PERFORMANCE);
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
