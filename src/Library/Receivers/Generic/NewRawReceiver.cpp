
/////////////////////////////////////////////////////////////
//
// Some handy utilities for creating GPS's and data streams.
//   Very leaky. Ok if only used for startup.
//   This will eventally be replaced with "RawReceiverUniversal"
//    which will manage memory properly.
//
// Need to figure out how to manage baudrates.
//
//////////////////////////////////////////////////////////////////

#include "NewRawReceiver.h"
#include "InputFile.h"
#include "OutputFile.h"
#include "StreamCopy.h"
#include "Rs232.h"

#include "RawTrimble.h"
#include "RawAC12.h"
#include "RawSirf.h"
#include "RawAntaris.h"
#include "RawAllstar.h"
#include "RawRtcm23.h"
#include "RawRtcm3.h"
#include "RawRinex.h"
#include "RawFuruno.h"
#include "RawSSF.h"
//#include "RawGarmin.h"
//#include "CommGarminUsb.h"
#include "CommWriteLog.h"
#include "CommReadLog.h"

RawReceiver* NewRawGarmin(const char* port, const char* raw);


RawReceiver* NewRawReceiver(const char* model, const char* port, const char* raw)
{
	debug("NewReceiver: model=%s  port=%s  raw=%s\n", model, port, raw);
	if (model == NULL) return NULL;

	//if (Same(model, "GPS18")) return NewRawGarmin(port, raw);

	Stream* s = NewInputStream(port, raw);
	if (s == NULL) return NULL;

	// process according to the model of receiver
	RawReceiver* gps = NULL;
	if      (Same(model, "AC12"))      gps = new RawAC12(*s); 
	else if (Same(model, "SIRF"))      gps = new RawSirf(*s);
	else if (Same(model, "LASSENIQ"))  gps = new RawTrimble(*s);
	else if (Same(model, "ANTARIS"))   gps = new RawAntaris(*s);
	else if (Same(model, "FURUNO"))    gps = new RawFuruno(*s);
	else if (Same(model, "ALLSTAR"))   gps = new RawAllstar(*s);
      else if (Same(model, "SSF"))       gps = new RawSSF(*s);
	else if (Same(model, "RTCM23"))      gps = new RawRtcm23(*s);
	else if (Same(model, "RTCM3"))      gps = new RawRtcm3(*s);
	else if (Same(model, "RTCM31"))      gps = new RawRtcm3(*s);
	else if (Same(model, "RINEX"))     gps = new RawRinex(*s);
	else if (Same(model, "XENIR"))    gps = new RawReverseRinex(*s);
	//else if (Same(model, "GPS18"))   gps = new RawGarmin(*s);
	else       Error("Didn't recognize receiver type %s", model);

	if (gps == NULL || gps->GetError() != OK) {
		Error("Unable to initialize GPS receiver %s\n", model);
		return NULL;
	}

	return gps;
}



Stream* NewOutputStream(const char* PortName)
{
	// If we succeed opening com port, then done
	Stream* port = new Rs232(PortName);
	if (port != NULL && port->GetError() == OK) return port;

	// Don't worry about com port errors. We'll retry as a file.
	if (port != NULL) delete port;
	ClearError();

	// If we succeed opening output file, then done
	port = new OutputFile(PortName);
	if (port != NULL && port->GetError() == OK) return port;

	// We had a problem
	if (port != NULL) delete port;
	Error("Unable to open output file %s\n", PortName);
	return NULL;
}
	

Stream* NewInputStream(const char* PortName, const char* RawFileName)
{
	// TODO: fix leaks on error exit.
	// Open the input, either a com port or a raw input file
	Stream* port = new Rs232(PortName);
	ClearError();
	if ( port == NULL || port->GetError() != OK)
		port = new InputFile(PortName);
	if (port == NULL || port->GetError() != OK) {
		Error("Unable to open the GPS raw file %s\n", PortName);
		return NULL;
	}

	// If we don't have a raw file, then done.
	if (RawFileName == NULL)
		return port;

	// Open the raw file for output
	Stream* raw = new OutputFile(RawFileName);
	if (raw == NULL || raw->GetError() != OK) {
		Error("Unable to open the 'raw' output file %s\n", RawFileName);
		return NULL;
	}

	// Create a "copy" stream which copies the input to the raw file
	Stream* copy = new StreamCopy(*port, *raw);
	if (copy == NULL || copy->GetError() != OK) {
		Error("Unable to copy data from port %s to raw file %s\n", port, raw);
		return NULL;
	}

	ClearError();
	return copy;
}




#ifdef NOTNOW

RawReceiver* NewRawGarmin(const char* port, const char* raw)
{
	// Get the input port, either the device or a raw file
	Comm* in;
	if (Same(port, "\\usb"))  
		in = new CommGarminUsbBuffered();
	else
	    in = new CommReadLog(port);
	debug("NewRawGarmin: in=%p  err=%d\n", in, in->GetError());
	if (in == NULL || in->GetError() != OK) {
		Error("Garmin couldn't open input port %s\n", port);
		return NULL;
	}

	// Create a raw output file if requested
	if (raw != NULL) {
		in = new CommWriteLog(*in, raw);
		debug("NewRawGarmin: in=%p err=%d\n", in, in->GetError());
		if (in == NULL || in->GetError() != OK) {
			Error("Unable to Create the Garmin Raw file %s\n", raw);
			return NULL;
		}
	}

	// Now, initialize a receiver
	RawReceiver* gps = new RawGarmin(*in);
	if (gps == NULL || gps->GetError() != OK) {
		Error("Couldn't initialize the Garmin receiver\n");
	    return NULL;
	}

	return gps;
}
#endif


