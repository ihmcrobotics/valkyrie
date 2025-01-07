package us.ihmc.valkyrie.perception;

public class ValkyrieHardwareAutonomyProcess
{
   private final ValkyrieAutonomyProcess perceptionAutonomyProcess;

   private ValkyrieHardwareAutonomyProcess()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::close, getClass().getSimpleName() + "Closer"));

      perceptionAutonomyProcess = new ValkyrieAutonomyProcess();
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
