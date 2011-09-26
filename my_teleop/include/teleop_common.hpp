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
#ifndef INCLUDE_TELEOP_COMMON_HPP
#define INCLUDE_TELEOP_COMMON_HPP
//=============================================================================




//=============================================================================
//Includes
//=============================================================================
#include <vector>




//=============================================================================
//Namespace
//=============================================================================
namespace teleop {




//=============================================================================
//Defines
//=============================================================================

/**@{ Axis range (0=off) */
#define TELEOP_AXIS_MIN                   -1.0
#define TELEOP_AXIS_MAX                   1.0
/**@}*/

/**@{ Axis types */
#define TELEOP_AXIS_TYPE_UNKNOWN          -1

#define TELEOP_AXIS_TYPE_LIN_X            1
#define TELEOP_AXIS_TYPE_LIN_Y            2
#define TELEOP_AXIS_TYPE_LIN_Z            3
#define TELEOP_AXIS_TYPE_ROT_X            4
#define TELEOP_AXIS_TYPE_ROT_Y            5
#define TELEOP_AXIS_TYPE_ROT_Z            6

#define TELEOP_AXIS_TYPE_THROTTLE         7
/**@}*/

/** Macro to determine axis type name */
#define TELEOP_AXIS_TYPE_NAME(x) (\
  ((x) == TELEOP_AXIS_TYPE_UNKNOWN        ) ? "TELEOP_AXIS_TYPE_UNKNOWN"        :\
  ((x) == TELEOP_AXIS_TYPE_LIN_X          ) ? "TELEOP_AXIS_TYPE_LIN_X"          :\
  ((x) == TELEOP_AXIS_TYPE_LIN_Y          ) ? "TELEOP_AXIS_TYPE_LIN_Y"          :\
  ((x) == TELEOP_AXIS_TYPE_LIN_Z          ) ? "TELEOP_AXIS_TYPE_LIN_Z"          :\
  ((x) == TELEOP_AXIS_TYPE_ROT_X          ) ? "TELEOP_AXIS_TYPE_ROT_X"          :\
  ((x) == TELEOP_AXIS_TYPE_ROT_Y          ) ? "TELEOP_AXIS_TYPE_ROT_Y"          :\
  ((x) == TELEOP_AXIS_TYPE_ROT_Z          ) ? "TELEOP_AXIS_TYPE_ROT_Z"          :\
  ((x) == TELEOP_AXIS_TYPE_THROTTLE       ) ? "TELEOP_AXIS_TYPE_THROTTLE"       :\
                                              "TELEOP_AXIS_TYPE_UNDEFINED"      )

/**@{ Button types */
#define TELEOP_BUTTON_TYPE_UNKNOWN        -1

#define TELEOP_BUTTON_TYPE_0              0
#define TELEOP_BUTTON_TYPE_1              1
#define TELEOP_BUTTON_TYPE_2              2
#define TELEOP_BUTTON_TYPE_3              3
#define TELEOP_BUTTON_TYPE_4              4
#define TELEOP_BUTTON_TYPE_5              5
#define TELEOP_BUTTON_TYPE_6              6
#define TELEOP_BUTTON_TYPE_7              7
#define TELEOP_BUTTON_TYPE_8              8
#define TELEOP_BUTTON_TYPE_9              9

#define TELEOP_BUTTON_TYPE_A              101
#define TELEOP_BUTTON_TYPE_B              102
#define TELEOP_BUTTON_TYPE_C              103
#define TELEOP_BUTTON_TYPE_X              124
#define TELEOP_BUTTON_TYPE_Y              125
#define TELEOP_BUTTON_TYPE_Z              126

#define TELEOP_BUTTON_TYPE_RIGHT          131
#define TELEOP_BUTTON_TYPE_LEFT           132

#define TELEOP_BUTTON_TYPE_SELECT         141
#define TELEOP_BUTTON_TYPE_START          142
#define TELEOP_BUTTON_TYPE_STOP           143
#define TELEOP_BUTTON_TYPE_TRIGGER        144
/**@}*/

/** Macro to determine button type name */
#define TELEOP_BUTTON_TYPE_NAME(x) (\
    ((x) == TELEOP_BUTTON_TYPE_UNKNOWN      ) ? "TELEOP_BUTTON_TYPE_UNKNOWN"      :\
    ((x) == TELEOP_BUTTON_TYPE_0            ) ? "TELEOP_BUTTON_TYPE_0"            :\
    ((x) == TELEOP_BUTTON_TYPE_1            ) ? "TELEOP_BUTTON_TYPE_1"            :\
    ((x) == TELEOP_BUTTON_TYPE_2            ) ? "TELEOP_BUTTON_TYPE_2"            :\
    ((x) == TELEOP_BUTTON_TYPE_3            ) ? "TELEOP_BUTTON_TYPE_3"            :\
    ((x) == TELEOP_BUTTON_TYPE_4            ) ? "TELEOP_BUTTON_TYPE_4"            :\
    ((x) == TELEOP_BUTTON_TYPE_5            ) ? "TELEOP_BUTTON_TYPE_5"            :\
    ((x) == TELEOP_BUTTON_TYPE_6            ) ? "TELEOP_BUTTON_TYPE_6"            :\
    ((x) == TELEOP_BUTTON_TYPE_7            ) ? "TELEOP_BUTTON_TYPE_7"            :\
    ((x) == TELEOP_BUTTON_TYPE_8            ) ? "TELEOP_BUTTON_TYPE_8"            :\
    ((x) == TELEOP_BUTTON_TYPE_9            ) ? "TELEOP_BUTTON_TYPE_9"            :\
    ((x) == TELEOP_BUTTON_TYPE_A            ) ? "TELEOP_BUTTON_TYPE_A"            :\
    ((x) == TELEOP_BUTTON_TYPE_B            ) ? "TELEOP_BUTTON_TYPE_B"            :\
    ((x) == TELEOP_BUTTON_TYPE_C            ) ? "TELEOP_BUTTON_TYPE_C"            :\
    ((x) == TELEOP_BUTTON_TYPE_X            ) ? "TELEOP_BUTTON_TYPE_X"            :\
    ((x) == TELEOP_BUTTON_TYPE_Y            ) ? "TELEOP_BUTTON_TYPE_Y"            :\
    ((x) == TELEOP_BUTTON_TYPE_Z            ) ? "TELEOP_BUTTON_TYPE_Z"            :\
    ((x) == TELEOP_BUTTON_TYPE_RIGHT        ) ? "TELEOP_BUTTON_TYPE_RIGHT"        :\
    ((x) == TELEOP_BUTTON_TYPE_LEFT         ) ? "TELEOP_BUTTON_TYPE_LEFT"         :\
    ((x) == TELEOP_BUTTON_TYPE_SELECT       ) ? "TELEOP_BUTTON_TYPE_SELECT"       :\
    ((x) == TELEOP_BUTTON_TYPE_START        ) ? "TELEOP_BUTTON_TYPE_START"        :\
    ((x) == TELEOP_BUTTON_TYPE_TRIGGER      ) ? "TELEOP_BUTTON_TYPE_TRIGGER"      :\
                                                "TELEOP_BUTTON_TYPE_UNDEFINED"    )




//=============================================================================
//Structs
//=============================================================================

/** Teleop device axis entry */
typedef struct {
  int type;
  float value;
} TeleopAxis;

/** Teleop device button entry */
typedef struct {
  int type;
  int value;
} TeleopButton;

/** Complete teleop device state */
typedef struct {
  std::vector<TeleopAxis> axes;
  std::vector<TeleopButton> buttons;
} TeleopState;




//=============================================================================
} //namespace
//=============================================================================




//=============================================================================
#endif //#ifndef INCLUDE_TELEOP_COMMON_HPP
//=============================================================================
