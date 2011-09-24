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
#include <boost/thread.hpp>
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
 * sources.  The start() and stop() methods start and stop a listening loop
 * which runs in a separate thread.  This loop listens for teleop device
 * events and reports them using the provided callback.  Sub-classes
 * for various teleop sources must implement the given pure virtual methods.
 */
class TeleopSource {

public:

  /**@{ Return values for listen method */
  static const int LISTEN_STATE_UNCHANGED = 1;
  static const int LISTEN_ERROR = -1;
  static const int LISTEN_STATE_CHANGED = 0;
  /**@}*/

  /**
   * Constructor.
   *
   *   @param callback [in] - callback to call with updated teleop state
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

  /** Timeout in seconds for listen (check for interruption this often) */
  static const int TIMEOUT_SECONDS = 1;

  /** Axis values smaller than this are set to 0.0 (sort of calibration) */
  static const float AXIS_THRESHOLD = 0.001;

  /**
   * Executes main listen loop.
   */
  void listenLoop();

  /**
   * Prepare to listen (open files, etc.).
   *
   *   @return true on success
   */
  virtual bool prepareToListen() = 0;

  /**
   * Blocks until teleop source device events detected or the given timeout is
   * reached.  When events occur, updates the teleop state.
   *
   *   @param teleop [in] - timeout value in seconds
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return -1 on error, 0 on timeout or no change to state, 1 on success
   */
  virtual int listen(int timeoutSeconds, TeleopState* teleop) = 0;

  /**
   * Done listening (close files, etc.).
   *
   *   @return true on success
   */
  virtual bool doneListening() = 0;

}; //class





//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_SOURCE_HPP
//=============================================================================
