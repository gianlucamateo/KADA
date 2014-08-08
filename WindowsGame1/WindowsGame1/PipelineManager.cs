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
        public ConcurrentQueue<PipelineContainer>[] processingQueues;
        private FrameCreator frameCreator;
        private PipelineDataContainer dataContainer;
        public ConcurrentQueue<PipelineContainer> recycle;
        public PipelineManager(PipelineDataContainer dataContainer)
        {
            this.recycle = new ConcurrentQueue<PipelineContainer>();
            

                this.processingQueues = new ConcurrentQueue<PipelineContainer>[8];
            for (int i = 0; i < this.processingQueues.Length; i++)
            {
                this.processingQueues[i] = new ConcurrentQueue<PipelineContainer>();
            }
            this.dataContainer = dataContainer;
            this.processor2D = new _2DProcessor(this, this.dataContainer);
            this.processor3D = new _3DProcessor(this,this.dataContainer);
            this.frameCreator = new FrameCreator(this.dataContainer,this);
            for (int i = 0; i < 50; i++)
            {
                recycle.Enqueue(new PipelineContainer(dataContainer));
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
                foreach (ConcurrentQueue<PipelineContainer> q in processingQueues)
                {
                    q.OrderBy(element => element.number);
                }
                Thread.Sleep(1);
            }
        }
        private void enqueue(PipelineContainer container)
        {
            int index = ++container.stage;
            if (index >= processingQueues.Length)
            {
                container = null;
                System.Diagnostics.Debug.WriteLine("Container reached end of pipeline");
                return;
            }
            if (processingQueues[index].Count < 20)
            {
                this.processingQueues[index].Enqueue(container);
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
            if (processingQueues[stage].Count > 1)
            {
                processingQueues[stage].TryDequeue(out p);
                return p;
            }
            else
                return null;
        }

        private void printStats()
        {
            while (this.dataContainer.run)
            {
                System.Diagnostics.Debug.Write("Usage:");
                for (int i = 0; i < this.processingQueues.Length; i++)
                {
                    System.Diagnostics.Debug.Write(" [" + i + "]: " +processingQueues[i].Count);
                }
                System.Diagnostics.Debug.Write("\n");
                System.Diagnostics.Debug.WriteLine("recycling: " + this.recycle.Count);
                Thread.Sleep(10000);
            }
        }
        
    }
}
