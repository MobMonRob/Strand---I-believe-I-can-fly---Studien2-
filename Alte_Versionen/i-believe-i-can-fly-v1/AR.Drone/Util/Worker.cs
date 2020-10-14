using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace AR.Drone.Util
{
    /// <summary>
    /// Class for implementing a WorkerThread, by just providing the necessary callbacks
    /// </summary>
    internal abstract class Worker : IDisposable
    {
        private bool _disposed = false;
        private bool _stopRequested = false;
        private Thread _thread;
        private Object lockObject = new Object();
        private readonly string _threadName;
        protected WorkFinishedEventArgs resultState = new WorkFinishedEventArgs(true);

        //Events
        internal delegate void WorkFinishedEventHandler(object sender, WorkFinishedEventArgs e);
        internal event WorkFinishedEventHandler WorkFinished;

        /// <summary>
        /// The code that should be executed as a thread.
        /// IMPORTANT: Set resultState-Attribute with the result state
        /// </summary>
        internal abstract void DoWork();

        /// <summary>
        /// Gets called when a stop is requested.
        /// Some unblocking work can be done inside this method
        /// </summary>
        internal abstract void OnStopRequest();

        internal Worker(string threadName)
        {
            this._threadName = threadName;
        }

        /// <summary>
        ///  Requests a stop of the thread
        /// </summary>
        internal void RequestStop()
        {
            lock (lockObject)
            {
                _stopRequested = true;
                if (IsAlive())
                {
                    OnStopRequest();
                }
            }
        }

        /// <summary>
        /// Checks if a stop was requested
        /// </summary>
        internal bool IsStopRequested()
        {
            return _stopRequested;
        }

        /// <summary>
        /// Method for handling the lifecycle of the thread.
        /// It fires an event, when the work is finshed
        /// </summary>
        private void RuntimeLifeCycle()
        {
            this.DoWork();
            if (null != this.WorkFinished)
            {
                this.WorkFinished(this, resultState);
            }
        }

        /// <summary>
        /// Starts this instance as a thread.
        /// If a thread is already running, the call to this method is ignored
        /// </summary>
        internal void StartAsThread()
        {
            if (_thread != null && _thread.IsAlive)
            {
                // Thread is already running
                return;
            }
            _thread = new Thread(this.RuntimeLifeCycle);
            _thread.Start();
            _thread.Name = this._threadName;

            while (!_thread.IsAlive)
            {
                Thread.Sleep(10);
            }
        }

        /// <summary>
        /// Calls the Thread.Join Method
        /// </summary>
        public void Join()
        {
            _thread.Join();
        }

        /// <summary>
        /// Gets a value indicating the execution status of this thread
        /// </summary>
        /// <returns></returns>
        internal bool IsAlive()
        {
            if (_thread == null)
                return false;

            return _thread.IsAlive;
        }
        
        ~Worker()
        {
            Dispose(false);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool isSaveToRemoveManagedObjects)
        {
            if (!_disposed)
            {
                if (isSaveToRemoveManagedObjects)
                {
                    // terminate thread first if it is alive
                    RequestStop();
                    this.Join();
                }
                _disposed = true;
            }
        }
    }
}