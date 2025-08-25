package us.ihmc.valkyrie.rdx;

import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedImageSensor;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.perception.ValkyrieAutonomyProcess;

public class ValkyrieRDXSimulatedAutonomyProcess
{
   private final RDXSimulatedImageSensor zedSensor;

   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   public ValkyrieRDXSimulatedAutonomyProcess(ValkyrieRobotModel robotModel)
   {
      zedSensor = RDXSimulatedSensorFactory.createZED2iImageSensor();

      perceptionAutonomyProcess = new ValkyrieAutonomyProcess(robotModel, zedSensor);
   }

   public void create(RDX3DScene scene)
   {
      zedSensor.create(scene);
   }

   public void render()
   {
      zedSensor.render();
   }

   public void close()
   {
      perceptionAutonomyProcess.close();
   }
}
