/*
 * Copyright (c) 2011, Kevin LeBlanc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */




//=============================================================================
#ifndef INCLUDE_TELEOP_SOURCE_HPP
#define INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <teleop_common.hpp>



//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/**
 * Callback used to report an updated teleop state.
 *
 *   @param teleopState [in] - The latest teleop state
 *
 *   @return true on success
 */
typedef bool (*TeleopSourceCallback)(TeleopState* teleopState);




//=============================================================================
//Classes
//=============================================================================

/**
 * This class provides a framework for generic handling of tele-operation
 * sources.  The start() method starts a listening loop in a separate thread.
 * The start method can optionally be blocking, in which case the listening
 * thread is joined before start returns.
 *
 * The pure virtual listen() method will be called in each iteration of the
 * listening loop.  This method must be implemented by a sub-class (i.e. a
 * teleop source).  The method should block until one or more teleop events
 * are detected.  Detected events should be used to update the current teleop
 * state.
 *
 * Once the state has been updated, the new state is passed into the should then be passed to the given callback.
 */
class TeleopSource
{

public:

  /**
   * Constructor.
   */
  TeleopSource(TeleopSourceCallback callback);

  /**
   * Destructor.
   */
  ~TeleopSource();

  /**
   * Start listening and publishing teleop device activity.
   *
   *   @param blocking [in] - true if this method should block
   *
   *   @return true on success
   */
  bool start(bool blocking);

  /**
   * Stop listening and publishing teleop device activity.  If start was called
   * in blocking mode, it returns after stop is called.  This method can also
   * be blocking or non-blocking.
   *
   *   @param blocking [in] - true if this method should block
   *
   *   @return true on success
   */
  bool stop(bool blocking);

  /**
   * Check if source is running (listening and publishing).
   *
   *   @return true if running.
   */
  bool isRunning();

private:

  /** Callback */
  TeleopSourceCallback mCallback;

  /** Listening thread */
  boost::thread mThread;

  /**
   * Blocks while listening for teleop source device events.  When events occur,
   * updates the teleop output status via the teleop parameter and returns.
   *
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return true on success
   */
  virtual bool listen(TeleopState* teleopState) = 0;

  /**
   * Executes main listen loop.
   *
   *   @return true on success
   */
  bool loop();

}; //class





//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================
