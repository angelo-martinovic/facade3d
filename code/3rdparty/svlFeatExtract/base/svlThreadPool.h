/*****************************************************************************
** STAIR VISION LIBRARY
** Copyright (c) 2007-2010, David Breeden
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the Stanford University nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
** EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
******************************************************************************
** FILENAME:    svlThreadPool.h
** AUTHOR(S):   David Breeden <breeden@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Stephen Gould <sgould@stanford.edu>
** DESCRIPTION:
**  Implements the thread pool pattern using PThreads.
**
*****************************************************************************/

#pragma once
#include <queue>

// TODO: make Windows-compatible
#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define USE_WINDOWSTHREAD
#else
#define USE_PTHREADS
#endif

#ifdef USE_PTHREADS
#include <pthread.h>
#endif

#ifdef USE_WINDOWSTHREAD
// TODO: include Windows Thread headers (windows.h, process.h) and use
// For now just hacking it by ignoring threads
#endif

using namespace std;

typedef void *(*thread_routine_t)(void *, unsigned tid);

// This class implements a thread pool, where N threads are contained and
// perform the jobs in a queue.  As soon as one of the N threads completes a
// job, it requests the next job in the queue, until the queue is empty, at
// which point it waits for a new job.
class svlThreadPool {
 public:
    static unsigned MAX_THREADS;

 public:
    svlThreadPool(const unsigned size = 2);  // Constructor
    ~svlThreadPool();                        // Destructor

    // Prep the threadpool to take jobs
    void start();

    // Add a job to the queue
    void addJob(thread_routine_t, void *arg);
    
    // Finish the jobs in the queue and stop
    void finish();
    
    // Main thread function
    static void *runJobs(void *argPtr);

    unsigned numThreads() { return _nThreads; }

private:

#ifdef USE_PTHREADS
    struct JobArgs {
        pthread_mutex_t* mutexPtr;
        pthread_cond_t* condPtr;
        queue<pair<thread_routine_t, void *> >* jobQPtr;
        bool* quitPtr;
        unsigned tid;
    };

    queue<pair<thread_routine_t, void *> > _jobQ;  // job queue
    pthread_t *_threads;     // bank of threads
    unsigned _nThreads;      // # of threads
    pthread_mutex_t _mutex;  // mutex to schedule threads
    pthread_cond_t _cond;    // condition variable to wait for jobs
    bool _bQuit;             // Off button
    vector<JobArgs*> _workerArgs;    // struct of references to shared resources
#else
    void *_threads;
    unsigned _nThreads;
#endif

};

