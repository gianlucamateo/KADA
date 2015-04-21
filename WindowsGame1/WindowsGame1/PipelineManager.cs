using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading;

namespace KADA
{
    public class PipelineManager
    {
        private _2DProcessor processor2D;
        public _3DProcessor processor3D;
        public ConcurrentQueue<PipelineContainer>[] ProcessingQueues;
        private FrameCreator frameCreator;
        private PipelineDataContainer dataContainer;
        public ConcurrentQueue<PipelineContainer> Recycle;
        public PipelineManager(PipelineDataContainer dataContainer)
        {
            this.Recycle = new ConcurrentQueue<PipelineContainer>();           

            this.ProcessingQueues = new ConcurrentQueue<PipelineContainer>[9];

            
            
            for (int i = 0; i < this.ProcessingQueues.Length; i++)
            {
                this.ProcessingQueues[i] = new ConcurrentQueue<PipelineContainer>();
            }
            this.dataContainer = dataContainer;
            this.processor2D = new _2DProcessor(this, this.dataContainer);
            this.processor3D = new _3DProcessor(this,this.dataContainer);
            this.frameCreator = new FrameCreator(this.dataContainer,this);
            for (int i = 0; i < 25; i++)
            {
                Recycle.Enqueue(new PipelineContainer(dataContainer));
            }
            Thread statPrinter = new Thread(new ThreadStart(() => this.printStats()));
            //statPrinter.Start();

            Thread cleaner = new Thread(new ThreadStart(() => this.clean()));
            //cleaner.Start();


            
        }

        private void clean()
        {
            
            while (true)
            {
                foreach (ConcurrentQueue<PipelineContainer> q in ProcessingQueues)
                {
                    q.OrderBy(element => element.Number);
                }
                Thread.Sleep(1);
            }
        }
        private void enqueue(PipelineContainer container)
        {
            int index = ++container.Stage;
            if (index >= ProcessingQueues.Length)
            {
                container = null;
                System.Diagnostics.Debug.WriteLine("Container reached end of pipeline");
                return;
            }
            if (ProcessingQueues[index].Count < 20)
            {
                this.ProcessingQueues[index].Enqueue(container);
            }
            else
            {
                container = null;
                System.Diagnostics.Debug.WriteLine("Discarding Container from bin " + index);
            }

        }

        private PipelineContainer dequeue(int stage)
        {
            PipelineContainer p = null;
            if (ProcessingQueues[stage].Count > 1)
            {
                ProcessingQueues[stage].TryDequeue(out p);
                return p;
            }
            else
                return null;
        }

        private void printStats()
        {
            while (this.dataContainer.Run)
            {
                System.Diagnostics.Debug.Write("Usage:");
                for (int i = 0; i < this.ProcessingQueues.Length; i++)
                {
                    System.Diagnostics.Debug.Write(" [" + i + "]: " +ProcessingQueues[i].Count);
                }
                System.Diagnostics.Debug.Write("\n");
                System.Diagnostics.Debug.WriteLine("recycling: " + this.Recycle.Count);
                Thread.Sleep(10000);
            }
        }
        
    }
}
