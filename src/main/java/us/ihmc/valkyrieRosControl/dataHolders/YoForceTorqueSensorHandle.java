package us.ihmc.valkyrieRosControl.dataHolders;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.rosControl.wholeRobot.ForceTorqueSensorHandle;

public class YoForceTorqueSensorHandle
{
   private final ForceTorqueSensorHandle handle;
   private final ForceSensorDefinition forceSensorDefinition;

   private final YoDouble tx, ty, tz;
   private final YoDouble fx, fy, fz;

   public YoForceTorqueSensorHandle(ForceTorqueSensorHandle handle, ForceSensorDefinition forceSensorDefinition, YoRegistry parentRegistry)
   {
      this.handle = handle;
      this.forceSensorDefinition = forceSensorDefinition;

      String name = forceSensorDefinition.getSensorName();
      YoRegistry registry = new YoRegistry(name);

      this.tx = new YoDouble(name + "_tx", registry);
      this.ty = new YoDouble(name + "_ty", registry);
      this.tz = new YoDouble(name + "_tz", registry);

      this.fx = new YoDouble(name + "_fx", registry);
      this.fy = new YoDouble(name + "_fy", registry);
      this.fz = new YoDouble(name + "_fz", registry);
   }

   public void update()
   {
      this.tx.set(handle.getTx());
      this.ty.set(handle.getTy());
      this.tz.set(handle.getTz());

      this.fx.set(handle.getFx());
      this.fy.set(handle.getFy());
      this.fz.set(handle.getFz());
   }

   public ForceSensorDefinition getForceSensorDefinition()
   {
      return forceSensorDefinition;
   }

   
   public void packWrench(DMatrixRMaj torqueForce)
   {
      torqueForce.set(0, tx.getDoubleValue());
      torqueForce.set(1, ty.getDoubleValue());
      torqueForce.set(2, tz.getDoubleValue());

      torqueForce.set(3, fx.getDoubleValue());
      torqueForce.set(4, fy.getDoubleValue());
      torqueForce.set(5, fz.getDoubleValue());
   }
}
