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
 * Callback to report an updated teleop state, or an error in the listening
 * thread.  This is called from the teleop source listening thread.
 *
 *   @param teleopState [in] - the latest teleop state
 *   @param error [in] - true if thread is stopping because of an error
 *
 *   @return true on success
 */
typedef bool (*TeleopSourceCallback)(TeleopState* teleopState, bool error);




//=============================================================================
//Classes
//=============================================================================

/**
 * This class provides a framework for generic handling of tele-operation
 * sources.  The start() and stop() methods start and stop a listening loop
 * which runs in a separate thread.  This loop listens for teleop device events
 * and reports them using the provided callback.
 *
 * Sub-classes for each teleop source must implement the given pure virtual
 * methods to perform the actual listening, as well as related preparation and
 * cleanup.
 *
 * The class is non-copyable as a precaution, since many teleop sources will
 * use members or resources which are difficult to share.  This could be
 * delegated to the individual sub-classes, but it's safer to do it here,
 * especially since copying a teleop source is not very useful at this point.
 */
class TeleopSource : boost::noncopyable {

public:

  /**@{ Default listen timeout in seconds - check for interruption this often */
  static const int LISTEN_TIMEOUT_DEFAULT = 1;
  static const int LISTEN_TIMEOUT_MIN = 0;
  static const int LISTEN_TIMEOUT_MAX = 3600;
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
   * Start listening thread which reports teleop device activity.
   *
   *   @return true on success
   */
  bool start();

  /**
   * Stop listening thread which reports teleop device activity.
   *
   *   @return true on success
   */
  bool stop();

  /**
   * Check if listening thread is running.
   *
   *   @return true if running.
   */
  bool isRunning();

  /**
   * Set listen timeout which specifies how often to check for interruption.
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
   * Set axis dead zone for all axes.
   *
   *   @param deadZone [in] - axis dead zone
   *
   *   @return true on success
   */
  bool setAxisDeadZoneForAllAxes(float axisDeadZone);

  /**
   * Set axis dead zone for a given axis.
   *
   *   @param axisDeadZone [in] - axis dead zone
   *   @param axisType [in] - axis type
   *
   *   @return true on success
   */
  bool setAxisDeadZone(float axisDeadZone, TeleopAxisType axisType);

  /**
   * Get axis dead zone for a given axis.
   *
   *   @param axisType [in] - axis type
   *
   *   @return axis dead zone
   */
  float getAxisDeadZone(TeleopAxisType axisType);

  /**
   * Set axis inverted status for all axes.
   *
   *   @param axisInverted [in] - axis inverted status
   *
   *   @return true on success
   */
  bool setAxisInvertedForAllAxes(bool axisInverted);

  /**
   * Set axis inverted status for a given axis.
   *
   *   @param axisInverted [in] - axis inverted status
   *   @param axisType [in] - axis type
   *
   *   @return true on success
   */
  bool setAxisInverted(bool axisInverted, TeleopAxisType axisType);

  /**
   * Get axis inverted status for a given axis.
   *
   *   @param axisType [in] - axis type
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

  /** Axis inverted statuses */
  bool mAxisInverted[TELEOP_AXIS_TYPE_COUNT];

  /** Listening thread */
  boost::thread mThread;

  /** Mutex for protecting changes to thread running status */
  boost::recursive_mutex mThreadMutex;

  /** Mutex for protecting listen timeout */
  boost::mutex mListenTimeoutMutex;

  /** Mutex for protecting axis dead zone */
  boost::mutex mAxisDeadZoneMutex;

  /** Mutex for protecting axis inverted */
  boost::mutex mAxisInvertedMutex;

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
   * listening thread, it must be careful when using modifiable class members.
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
