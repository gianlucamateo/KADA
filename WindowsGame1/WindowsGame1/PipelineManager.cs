using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;

namespace KADA
{
    public class PipelineManager
    {
        private _2DProcessor processor2D;
        private _3DProcessor processor3D;
        public ConcurrentQueue<PipelineContainer>[] processingQueues;
        private FrameCreator frameCreator;
        private PipelineDataContainer dataContainer;
        public ConcurrentQueue<PipelineContainer> recycle;
        public PipelineManager(PipelineDataContainer dataContainer)
        {
            this.recycle = new ConcurrentQueue<PipelineContainer>();
            

                this.processingQueues = new ConcurrentQueue<PipelineContainer>[7];
            for (int i = 0; i < this.processingQueues.Length; i++)
            {
                this.processingQueues[i] = new ConcurrentQueue<PipelineContainer>();
            }
            this.dataContainer = dataContainer;
            this.processor2D = new _2DProcessor(this, this.dataContainer);
            this.processor3D = new _3DProcessor(this,this.dataContainer);
            this.frameCreator = new FrameCreator(this.dataContainer,this);
            for (int i = 0; i < 20; i++)
            {
                recycle.Enqueue(new PipelineContainer(dataContainer));
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
        
    }
}
