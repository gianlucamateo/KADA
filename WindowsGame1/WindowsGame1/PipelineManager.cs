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
        private ConcurrentQueue<PipelineContainer>[] processingQueues;
        private FrameCreator frameCreator;
        private PipelineDataContainer dataContainer;
        public PipelineManager()
        {
            this.processingQueues = new ConcurrentQueue<PipelineContainer>[7];
            for (int i = 0; i < this.processingQueues.Length; i++)
            {
                this.processingQueues[i] = new ConcurrentQueue<PipelineContainer>();
            }
            this.dataContainer = new PipelineDataContainer();
            this.processor2D = new _2DProcessor(this);
            this.processor3D = new _3DProcessor(this,this.dataContainer);
            
            
            this.frameCreator = new FrameCreator(this.dataContainer,this);
            
        }

        public void enqueue(PipelineContainer container)
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

        public PipelineContainer dequeue(int stage)
        {
            PipelineContainer p = null;
            processingQueues[stage].TryDequeue(out p);
            return p;
        }
    }
}
