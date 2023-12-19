package us.ihmc.valkyrie.encoder.comparison;

import java.io.IOException;
import java.util.LinkedHashMap;

import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.sensorProcessing.encoder.processors.EncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.JerryEncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.JerryEncoderProcessorNoYoVariablesWrapper;
import us.ihmc.sensorProcessing.encoder.processors.NaiveEncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.NonlinearObserverEncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.PolynomialFittingEncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.StateMachineEncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.StateMachineSimpleEncoderProcessor;
import us.ihmc.sensorProcessing.encoder.processors.StateMachineTwoEncoderProcessor;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class EncorderProcessorEvalRealData
{
   private final YoRegistry registry;
   private final LinkedHashMap<EncoderProcessor, String> encoderProcessors = new LinkedHashMap<EncoderProcessor, String>();
   private final LinkedHashMap<EncoderProcessor, YoDouble> processedPositions = new LinkedHashMap<EncoderProcessor, YoDouble>();
   private final LinkedHashMap<EncoderProcessor, YoDouble> processedRates = new LinkedHashMap<EncoderProcessor, YoDouble>();
   private final YoInteger rawTicks, heartBeat;
   private final YoDouble rawPosition;
   private final YoDouble time;
   private final YoDouble turboJerryRate,fdRate;
   private final YoBuffer dataBuffer;

   private final double dt;
   private final RealLifeEncoderTrajectory encoder;
   private final Thread simThread;

   public EncorderProcessorEvalRealData() throws IOException
   {
      Robot nullRobot = new Robot("nullRobot");
      registry = nullRobot.getRobotsYoRegistry();
      rawTicks = new YoInteger("rawTicks", registry);
      rawPosition = new YoDouble("rawPosition", registry);
      heartBeat = new YoInteger("turboHeartBeat", registry);
      
      turboJerryRate = new YoDouble("turboJerryProcRate", registry);
      fdRate = new YoDouble("fdRate", registry);
      time = nullRobot.getYoTime();
      this.encoder = new RealLifeEncoderTrajectory("data/JointAPSData.m","trunk.wj2");

      this.dt = encoder.getDt();
      

      double distancePerTick = 2*Math.PI / encoder.tickPerRev();
      encoderProcessors.put(new StateMachineEncoderProcessor("StateMachine", rawTicks, time, distancePerTick, registry), "StateMachine");
      encoderProcessors.put(new StateMachineSimpleEncoderProcessor("StateMachineSimple", rawTicks, time, distancePerTick, registry), "StateMachineSimple");
      encoderProcessors.put(new StateMachineTwoEncoderProcessor("StateMachineTwo", rawTicks, time, distancePerTick, registry), "StateMachineTwo");
      encoderProcessors.put(new NonlinearObserverEncoderProcessor("NonlinObserver", rawTicks, time, distancePerTick, registry), "NonlinObserver");
      encoderProcessors.put(new NaiveEncoderProcessor("Naive", rawTicks, time, distancePerTick, registry), "Naive");
      encoderProcessors.put(new PolynomialFittingEncoderProcessor("PolyFit320", rawTicks, time, distancePerTick, 3, 2, 0, registry), "PolyFit320");
      encoderProcessors.put(new PolynomialFittingEncoderProcessor("PolyFit321", rawTicks, time, distancePerTick, 3, 2, 1, registry), "PolyFit321");
      encoderProcessors.put(new PolynomialFittingEncoderProcessor("PolyFit532", rawTicks, time, distancePerTick, 5, 3, 2, registry), "PolyFit532");
      encoderProcessors.put(new PolynomialFittingEncoderProcessor("PolyFit820", rawTicks, time, distancePerTick, 8, 2, 0, registry), "PolyFit820");
      encoderProcessors.put(new JerryEncoderProcessor("Jerry", rawTicks, time, distancePerTick, dt, registry), "Jerry");
      encoderProcessors.put(new JerryEncoderProcessor("JerrySlow", rawTicks, time, distancePerTick, dt, 10,registry), "JerrySlow");
      encoderProcessors.put(new JerryEncoderProcessorNoYoVariablesWrapper("JerryNoYo", rawTicks, time, distancePerTick, dt), "JerryNoYo");


      for (EncoderProcessor encoderProcessor : encoderProcessors.keySet())
      {
         processedPositions.put(encoderProcessor, new YoDouble("p_" + encoderProcessors.get(encoderProcessor), registry));
         processedRates.put(encoderProcessor, new YoDouble("pd_" + encoderProcessors.get(encoderProcessor), registry));
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(encoder.getNumTicks() + 1);
      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot, parameters);
      scs.hideViewport();
      dataBuffer = scs.getDataBuffer();
      simThread = new Thread(scs, "R2Simulation sim thread");
   }

   public void start()
   {
      simThread.start();

      for (EncoderProcessor encoderProcessor : encoderProcessors.keySet())
      {
         encoderProcessor.initialize();
      }

      double prevPosition=encoder.getPosition();
      for(int i=0; i< encoder.getNumTicks();i++)
      {  
         
            time.set(encoder.getTime());
            heartBeat.set(encoder.getHeartBeat());
            rawTicks.set(encoder.getUnwrappedEncoderTicks());
            rawPosition.set(encoder.getPosition());
            turboJerryRate.set(encoder.getVelocity());
            fdRate.set((encoder.getPosition()-prevPosition)/dt);

            for (EncoderProcessor processor : encoderProcessors.keySet())
            {
               processor.update();
               processedPositions.get(processor).set(processor.getQ());
               processedRates.get(processor).set(processor.getQd());
            }

            dataBuffer.tickAndWriteIntoBuffer();
            prevPosition=encoder.getPosition();
            encoder.nextTimestep();
      }
   }

   public static void main(String[] args) throws IOException
   {
      EncorderProcessorEvalRealData comparer = new EncorderProcessorEvalRealData();
      comparer.start();
   }
}
