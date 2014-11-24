/*****************************************************************************
/*****************************************************************************
 * Copyright (C)  07 Nov 2014 Simon Derek Hughes
 *
 * debug.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  For more information, contact Simon Hughes at:
 *    Email:    simon.d.hughes@arisena.com
 *    Address:  Arisena, 11 Gough Way, Cambridge, CB3 9LN, UK.
 *
 *****************************************************************************
 * DESCRIPTION:
 *  Debug support. Define the symbol _DEBUG in CFLAGS to enable debug trace
 *  message output to syslog.
 *  Linux
 *   Implementation depends on compiler defining linux or __linux. Define
 *   these symbols if the compiler doesn't automatically do so.
 *****************************************************************************
 * MODIFICATION HISTORY
 * 20141111  SDH   Created.
 *****************************************************************************/

#ifndef _RPI_I2S_TEST_DEBUG_H_
#define _RPI_I2S_TEST_DEBUG_H_

#include <stdio.h>

#define LOG_NONE        0
#define LOG_ERR         1
#define LOG_WARN        2
#define LOG_NOTICE      3
#define LOG_INFO        4
#define LOG_DEBUG       5

extern int rpi_optDebug;
extern int rpi_optLogLevel;

void syslog(int priority, const char* format, ...);


#ifdef _DEBUG
#define RPII2S_ASSERT(_x)                   assert(_x)

#ifdef WIN32
/* windows form of variadic macro */
#define RPII2S_DBGLOG(_fmt, ...)                        \
  do                                                    \
  {                                                     \
    if(rpi_optDebug && (rpi_optLogLevel >= LOG_DEBUG))  \
    {                                                   \
        printf(_fmt, __VA_ARGS__);                      \
    }                                                   \
  }while(0);

#endif /* WIN32 */

#if defined linux || defined __linux
#define RPII2S_DBGLOG(_fmt, args...)                    \
  do                                                    \
  {                                                     \
    if(rpi_optDebug && (rpi_optLogLevel >= LOG_DEBUG))  \
    {                                                   \
      syslog(LOG_DEBUG, _fmt, ##args);                  \
    }                                                   \
  } while(0);

#endif /* linux  */

#else
#define RPII2S_ASSERT(_x)                   do { } while(0)
#define RPII2S_DBGLOG(_fmt, ...)            do { } while(0)
#endif /* _DEBUG */



#ifdef WIN32
/* windows form of variadic macro */
#define RPII2S_ERRLOG(_fmt, ...)                        \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_ERR)                      \
    {                                                   \
        printf(_fmt, __VA_ARGS__);                      \
    }                                                   \
  }while(0);

#define RPII2S_WARN(_fmt, ...)                          \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_WARN)                     \
    {                                                   \
        printf(_fmt, __VA_ARGS__);                      \
    }                                                   \
  }while(0);

#define RPII2S_NOTICE(_fmt, ...)                        \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_NOTICE)                   \
    {                                                   \
        printf(_fmt, __VA_ARGS__);                      \
    }                                                   \
  }while(0);

#define RPII2S_INFO(_fmt, ...)                          \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_INFO)                     \
    {                                                   \
        printf(_fmt, __VA_ARGS__);                      \
    }                                                   \
  }while(0);

#endif /* WIN32 */

#if defined linux || defined __linux
#define RPII2S_ERRLOG(_fmt, args...)                    \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_ERR)                      \
    {                                                   \
      syslog(LOG_ERR, _fmt, ##args);                    \
    }                                                   \
  } while(0);

#define RPII2S_WARN(_fmt, args...)                      \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_WARN)                     \
    {                                                   \
      syslog(LOG_WARN, _fmt, ##args);                   \
    }                                                   \
  } while(0);

#define RPII2S_NOTICE(_fmt, args...)                    \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_NOTICE)                   \
    {                                                   \
      syslog(LOG_NOTICE, _fmt, ##args);                 \
    }                                                   \
  } while(0);

#define RPII2S_INFO(_fmt, args...)                      \
  do                                                    \
  {                                                     \
    if(rpi_optLogLevel >= LOG_INFO)                     \
    {                                                   \
      syslog(LOG_INFO, _fmt, ##args);                   \
    }                                                   \
  } while(0);

#endif /* linux || __linux */


#endif /* _RPI_I2S_TEST_DEBUG_H_ */
