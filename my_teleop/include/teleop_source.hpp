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
#include <boost/noncopyable.hpp>
#include <boost/thread.hpp>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Types
//=============================================================================

/** Return values for listen method */
typedef enum {
  LISTEN_RESULT_ERROR,
  LISTEN_RESULT_UNCHANGED,
  LISTEN_RESULT_CHANGED
} ListenResult;

/**
 * Callback used to report an updated teleop state.  This is called from the
 * teleop source listening thread.
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
 * which runs in a separate thread.  This loop listens for teleop device events
 * and reports them using the provided callback.  Sub-classes for each teleop
 * source must implement the given pure virtual methods.
 *
 * The class is non-copyable as a precaution, since some teleop sources may
 * use members or resources which are difficult to share.
 */
class TeleopSource : boost::noncopyable {

public:

  /**@{ Default listen timeout in seconds - check for interruption this often */
  static const int LISTEN_TIMEOUT_DEFAULT = 1;
  static const int LISTEN_TIMEOUT_MIN = 0;
  static const int LISTEN_TIMEOUT_MAX = 60;
  /**@}*/

  /**@{ Axis dead zone - values smaller than this are set to 0.0 */
  static const float AXIS_DEAD_ZONE_DEFAULT = 0.01;
  static const float AXIS_DEAD_ZONE_MIN = 0.01;
  static const float AXIS_DEAD_ZONE_MAX = 0.99;
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
   * Start listening for teleop device activity.
   *
   *   @param blocking [in] - true if should block until thread has stopped
   *
   *   @return true on success
   */
  bool start(bool blocking);

  /**
   * Stop listening for teleop device activity.
   *
   *   @param blocking [in] - true if should block until thread has stopped
   *
   *   @return true on success
   */
  bool stop(bool blocking);

  /**
   * Check if source is running (listening).
   *
   *   @return true if running.
   */
  bool isRunning();

  /**
   * Set listen timeout, which determines how often the listening thread checks
   * for interruption.  Should only be called when listening thread is not
   * running.
   *
   *   @param listenTimeout [in] - listen timeout in seconds
   *
   *   @return true on success
   */
  bool setListenTimeout(int listenTimeout);

  /**
   * Get listen timeout.
   *
   *   @return listen timeout
   */
  int getListenTimeout();

  /**
   * Set axis dead zone for all axes.  Should only be called when listening
   * thread is not running.
   *
   *   @param deadZone [in] - axis dead zone
   *
   *   @return true on success
   */
  bool setAxisDeadZoneForAllAxes(float axisDeadZone);

  /**
   * Set axis dead zone.  Should only be called when listening thread is not
   * running.
   *
   *   @param deadZone [in] - axis dead zone
   *   @param axisType [in] - axis type to set
   *
   *   @return true on success
   */
  bool setAxisDeadZone(float axisDeadZone, TeleopAxisType axisType);

  /**
   * Get axis dead zone.
   *
   *   @param axisType [in] - axis type to check
   *
   *   @return axis dead zone
   */
  float getAxisDeadZone(TeleopAxisType axisType);

  /**
   * Set axis inverted status for all axes.  Should only be called when
   * listening thread is not running.
   *
   *   @param inverted [in] - inverted status
   *
   *   @return true on success
   */
  bool setAxisInvertedForAllAxes(bool axisInverted);

  /**
   * Set axis inverted status.  Should only be called when listening thread is
   * not running.
   *
   *   @param axisInverted [in] - axis inverted status
   *   @param axisType [in] - axis type to set
   *
   *   @return true on success
   */
  bool setAxisInverted(bool axisInverted, TeleopAxisType axisType);

  /**
   * Get axis inverted.
   *
   *   @param axisType [in] - axis type to check
   *
   *   @return axis inverted
   */
  bool getAxisInverted(TeleopAxisType axisType);

private:

  /** Callback */
  TeleopSourceCallback mCallback;

  /** Timeout in seconds for listen (check for interruption this often) */
  int mListenTimeout;

  /** Axis dead zones */
  float mAxisDeadZone[TELEOP_AXIS_TYPE_COUNT];

  /** Axis inverted */
  bool mAxisInverted[TELEOP_AXIS_TYPE_COUNT];

  /** Listening thread */
  boost::thread mThread;

  /** Mutex for protecting changes to thread running status */
  boost::recursive_mutex mMutex;

  /**
   * Executes main listen loop (run in separate thread).
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
   * reached.  When events occur, updates the teleop state.  The timeout could
   * also be inherited, but the interface is cleaner if sub-classes don't need
   * to worry about inherited members.  Since this method is called from the
   * listening thread, care should be taken if this method uses members which
   * can be modified after object creation (e.g. only allow members to be
   * updated while the listening thread is not running -- isRunning() can be
   * used to check if the listening thread is running).
   *
   *   @param timeoutSeconds [in] - timeout value in seconds
   *   @param teleop [in/out] - the current teleop output, to be updated
   *
   *   @return LISTEN_ERROR on error, LISTEN_STATE_UNCHANGED on timeout or no
   *           change to state, LISTEN_STATE_CHANGED if state updated
   */
  virtual ListenResult listen(int timeoutSeconds, TeleopState* const teleop) = 0;

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
