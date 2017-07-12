/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s): Peter Kazanzides, Anton Deguet
  Created on: 2006

  (C) Copyright 2007-2011 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsMedtronicStealthlink_AsCL_Stuff_h
#define _mtsMedtronicStealthlink_AsCL_Stuff_h

#ifdef CISST_HAS_STEALTHLINK
#if (CISST_OS == CISST_WINDOWS)
#include <AsCL_Base_Stuff.h>
#else
#include <AsCL/AsCL_Base_Stuff.h>
#endif
#endif

class osaStopwatch;

//------------------------------------------------------------------------------
class mtsMedtronicStealthlink_AsCL_IO_Watch: public AsCL_IO_Watch
{
public:

    mtsMedtronicStealthlink_AsCL_IO_Watch();
    virtual ~mtsMedtronicStealthlink_AsCL_IO_Watch();

    virtual int AddWatch(int, void *, void *);
    virtual void RemoveWatch(void);
    void CheckWatch(void);

private:

    typedef void (*mtsMedtronicStealthlink_CallBack)(int, void *);
    typedef void (*Watch_Callback)(void *);

    int fd;
    Watch_Callback Callback;
    void *ClientPtr;
};


//------------------------------------------------------------------------------
class mtsMedtronicStealthlink_AsCL_Timeout: public AsCL_Timeout
{
public:
    mtsMedtronicStealthlink_AsCL_Timeout();
    ~mtsMedtronicStealthlink_AsCL_Timeout();

    // Units of tmo_val?  This implementation assumes milliseconds.
    virtual int  AddTimeout(int tmo_val, void * func, void * obj);
    virtual void RemoveTimeout(void);
    void CheckTimeout(void);

private:
    typedef int (*Timeout_Callback) (void * data);

    osaStopwatch *StealthlinkTimer;

    double Timeout;   // seconds
    Timeout_Callback Callback;
    void *DataObj;
};


//------------------------------------------------------------------------------
class mtsMedtronicStealthlink_AsCL_Utils: public AsCL_Utils
{
    mtsMedtronicStealthlink_AsCL_IO_Watch *curWatch;
    mtsMedtronicStealthlink_AsCL_Timeout *curTimeout;

public:
    mtsMedtronicStealthlink_AsCL_Utils();
    virtual ~mtsMedtronicStealthlink_AsCL_Utils();

    virtual AsCL_IO_Watch * new_IO_Watch(void);
    virtual AsCL_Timeout * new_Timeout(void);

    void CheckCallbacks(void);
};

/*class mtsMedtronicStealthlink_AsCL_Utils: public AsCL_Utils
{

};*/


#endif // _mtsMedtronicStealthlink_AsCL_Stuff_h
