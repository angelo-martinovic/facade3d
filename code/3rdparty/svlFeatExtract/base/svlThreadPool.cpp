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
** FILENAME:    svlThreadPool.cpp
** AUTHOR(S):   David Breeden <breeden@cs.stanford.edu>
**              Ian Goodfellow <ia3n@cs.stanford.edu>
**              Stephen Gould <sgould@stanford.edu>
**
*****************************************************************************/

#include <iostream>
#include <limits>

#include "svlLogger.h"
#include "svlConfigManager.h"
#include "svlThreadPool.h"

// Static members
unsigned svlThreadPool::MAX_THREADS = 0;

// Constructor
svlThreadPool::svlThreadPool(const unsigned size) :
    _threads(NULL), _nThreads(0) {

#ifdef USE_PTHREADS
    _nThreads = (size > MAX_THREADS) ? MAX_THREADS : size;
    if (_nThreads > 0)
        _threads = new pthread_t[_nThreads];
    pthread_mutex_init(&_mutex, NULL);
    pthread_cond_init(&_cond, NULL);

    for (unsigned i = 0; i < _nThreads; i++)
        _workerArgs.push_back(new JobArgs);
#endif

}

// Destructor
svlThreadPool::~svlThreadPool() {

#ifdef USE_PTHREADS
    // tell threads to stop
    if (!_bQuit) {
        _bQuit = true;
        pthread_mutex_lock(&_mutex);
        pthread_cond_broadcast(&_cond);
        pthread_mutex_unlock(&_mutex);

        // wait for them to finish
        for (unsigned i = 0; i < _nThreads; i++) {
            pthread_join(_threads[i], NULL);
        }
    }

    pthread_mutex_destroy(&_mutex);
    pthread_cond_destroy(&_cond);
    if (_threads != NULL)
        delete[] _threads;
    for (unsigned i = 0; i < _workerArgs.size(); i++)
        delete _workerArgs[i];
#endif

}

// Spawn threads to take jobs
void svlThreadPool::start() {

#ifdef USE_PTHREADS
    _bQuit = false;

    for (unsigned i = 0; i < _nThreads; i++) {

        _workerArgs[i]->mutexPtr = &_mutex;
        _workerArgs[i]->condPtr = &_cond;
        _workerArgs[i]->jobQPtr = &_jobQ;
        _workerArgs[i]->quitPtr = &_bQuit;
        _workerArgs[i]->tid = i;

        pthread_create(&_threads[i], NULL, runJobs, _workerArgs[i]);
    }
#endif

}

// Add a new job to the job queue
void svlThreadPool::addJob(thread_routine_t routine, void *arg) {

#ifdef USE_PTHREADS
    if (_nThreads > 0) {
        // Acquire lock on queue
        pthread_mutex_lock(&_mutex);

        // push job on queue
        _jobQ.push(make_pair(routine, arg));

        // Tell threads about it
        pthread_cond_broadcast(&_cond);

        // unlock queue
        pthread_mutex_unlock(&_mutex);
    } else {
        // Just do it in the main thread.
        routine(arg, 0);
    }
#endif

#ifdef USE_WINDOWSTHREAD
    // Just do it in the main thread.  TODO: actually implement
    routine(arg, 0);
#endif
}

// Wait until jobs are finished, then return
void svlThreadPool::finish() {

#ifdef USE_PTHREADS
    while (true) {

        // Acquire lock on queue
        pthread_mutex_lock(&_mutex);

        // if it's empty, stop looping
        if (_jobQ.empty()) {
            break;
        }

        // if not, let the other threads get to it and try again
        pthread_mutex_unlock(&_mutex);
    }

    // tell the threads to quit
    _bQuit = true;
    pthread_cond_broadcast(&_cond);
    pthread_mutex_unlock(&_mutex);

    // Now wait for them to be done
    for (unsigned i = 0; i < _nThreads; i++) {
        pthread_join(_threads[i], NULL);
    }
#endif

}

// Thread main function
void *svlThreadPool::runJobs(void *argPtr) {

#ifdef USE_PTHREADS
    JobArgs *args = (JobArgs*) argPtr;

    // Keep asking for jobs until quit flag
    while (!*(args->quitPtr)) {

        // Wait for job
        pthread_mutex_lock(args->mutexPtr);
        while (args->jobQPtr->empty() && !*(args->quitPtr)) {
            pthread_cond_wait(args->condPtr, args->mutexPtr);
        }

        if (args->jobQPtr == NULL || args->jobQPtr->empty()) {
            pthread_mutex_unlock(args->mutexPtr);
            continue;
        }

        // Take job off queue
        pair<thread_routine_t, void *> job = args->jobQPtr->front();
        args->jobQPtr->pop();

        // unlock queue
        pthread_mutex_unlock(args->mutexPtr);

        // Do job
        job.first(job.second, args->tid);
    }
#endif

    return NULL;
}

// configuration --------------------------------------------------------

class svlThreadPoolConfig : public svlConfigurableModule {
public:
    svlThreadPoolConfig() : svlConfigurableModule("svlBase.svlThreadPool") { }
    ~svlThreadPoolConfig() { }

    void usage(ostream &os) const {
        os << "      threads      :: maximum number of concurrent threads (default: " 
           << svlThreadPool::MAX_THREADS << ")\n";
    }

    void setConfiguration(const char *name, const char *value) {
        // number of threads
        if (!strcmp(name, "threads")) {
            svlThreadPool::MAX_THREADS = atoi(value);
        } else {
            SVL_LOG(SVL_LOG_FATAL, "unrecognized configuration option for " << this->name());
        }
    }
};

static svlThreadPoolConfig gThreadPoolConfig;
