// Kinematic - a program for doing double difference, kinematic positioning
//    Part of kinematic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinematic@coyotebush.net
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, version 2.

//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#include "RawReceiver.h"  
#include "RawSimulator.h"
#include "DoubleDiff.h"
#include "SP3.h" 
#include "NewRawReceiver.h"
#include "OutputFile.h"
#include "Logger.h"
#include <stdio.h>
#include <stdarg.h>


class Formatter : public OutputFile
{
protected:
	static const int MaxItems = 20;
	int Precision[MaxItems];
	int NrFields;
    int FieldWidth;
	static const int TimeWidth=19;
	const char* Separator;

public:
	Formatter(const char* OutputFileName, int nrfields, const char**labels, const int* precisions,
		int fieldwidth=15, const char* separator=" ");
	virtual bool Data(double p1, ...);
	virtual ~Formatter();

private:
    bool Header(const char** labels);
	bool Numeric(double value, int width, int precision);
	bool Centered(const char* str, int width);
};

class PositionFormatter : public Formatter
{
public:
	PositionFormatter(const char* OutputFileName, int positiontype, int outputtype);
	~PositionFormatter();
};



class ResidualFile : public OutputFile
{
public:
	ResidualFile(const char* FileName) : OutputFile(FileName){}
	bool PrintResiduals(DoubleDiff& dbl);
};

bool ResidualFile::PrintResiduals(DoubleDiff& dbl)
{
	// Display the time
	if (PrintTime(dbl.GetTime()) != OK) return Error();

	// If we don't have a solution, show a message
	if (dbl.GetCep() == -1)
		return Printf("   *** No Solution ***\n");

	// Do for each satellite with data
	for (int s=0; s<MaxSats; s++) {
		if (!dbl.ValidPhase(s) && ! dbl.ValidCode(s)) continue;

		// display the satellite and it's code and/or phase
		if (dbl.ValidCode(s))  Printf(" C%d(%.3f)", s, abs(dbl.GetCodeResidual(s)));
		if (dbl.ValidPhase(s)) Printf(" P%d(%.3f)", s, abs(dbl.GetPhaseResidual(s)));
	}
	Printf("\n");
	return OK;
}





bool Process(int argc, const char** argv);
bool Configure(int argc, const char** argv);
bool DisplayOptions();


// run string parameters
static const char* BaseModel;
static const char* BasePortName;
static const char* RovingModel;
static const char* RovingPortName;
static const char* Sp3Name;
static const char* OutputName;
static enum {SPACES, COMMAS} OutputType;
static bool Static;
extern bool CodeOnly;
extern int DebugLevel;
static enum {WGS84, ECEF, ENU, TEST} PositionType;
static bool Simulator;



int main(int argc, const char** argv)
{
	Process(argc, argv);
	ShowErrors();
	return 0;
}

bool Process(int argc, const char** argv)
{
	// parse the command line
	if (Configure(argc, argv) != OK) {
		DisplayOptions();
		return Error();
	}

        double MinRange = 9999e99;
        double MaxRange = -9999e99;

	// Open up the two sources of raw measurements
	RawReceiver* base = NewRawReceiver(BaseModel, BasePortName);
	RawReceiver* roving = NewRawReceiver(RovingModel, RovingPortName);
	if (base == NULL || roving == NULL) return Error();

	// Read the first epoch so we have initial position estimates
	if (Range(base->Pos) == 0 && base->NextEpoch() != OK) return Error("Can't read first epoch from base\n");
	if (Range(roving->Pos) == 0 && roving->NextEpoch() != OK) return Error("Can't read first epoch from rover\n");

	// Configure the event logger to use base receiver's time clock
	EventSetTime(&base->GpsTime);
 
	// By default, we'll get the ephemerides from the base unit
	Ephemerides *eph = base;

	// Open the sp3 file if given
	if (Sp3Name != NULL)
		eph = new SP3(Sp3Name);
	if (eph == NULL) return Error();

	// Open the output file
	PositionFormatter Output(OutputName, PositionType, OutputType);
	if (Output.GetError() != OK) return Error("Can't open output file %s\n", OutputName);

	// Log the residuals
	ResidualFile Residuals("residuals.txt");
	if (Residuals.GetError() != OK) return Error("Can't open residuals file %s\n","residuals.txt");

	// Create a double difference engine
	DoubleDiff dbl(*eph, *base, *roving);
	if (Static)
		dbl.BeginStatic();

	// Setup ENU coordinates centered at the base station
	LocalEnu BaseCentered(base->Pos);
	LocalEnu RovingCentered(roving->Pos);

	Time time;
	Position pos;
	double cep;
	double fit;

	// do for each position until "done"
	while (dbl.NextPosition(time, pos, cep, fit) == OK) {

		// Convert the ECEF position to the desired form
		Triple triple;
		if      (PositionType == ECEF)     triple = pos;
		else if (PositionType == WGS84)    triple = PositionToWgs84(pos);
		else if (PositionType == ENU)      triple = BaseCentered.ToEnu(pos);
		else if (PositionType == TEST)     triple = RovingCentered.ToEnu(pos);

		// Display the position
		Output.PrintTime(base->GpsTime);
		if (cep == -1)   Output.Write("  *** No Data ***\n");
		else             Output.Data(triple[0], triple[1], triple[2], cep, fit);

		Residuals.PrintResiduals(dbl);

                MinRange = min(MinRange, Range(roving->Pos-base->Pos));
                MaxRange = max(MaxRange, Range(roving->Pos-base->Pos));
	}

        printf("The base ranged from %.3f km to %.3f km\n", MinRange, MaxRange);
        debug ("The base ranged from %.3f km to %.3f km\n", MinRange, MaxRange);

	return OK;
}



 bool Configure(int argc, const char** argv)
 {
     // defaults
	 CodeOnly = false;
	 Static = false;
	 Sp3Name = NULL;
	 OutputName = NULL;
	 OutputType = SPACES;
	 Simulator = false;

	 // Do for each argument
	 const char* arg;
	 int i;
	 for (i=1; i<argc && argv[i][0] == '-'; i++) {

		 if (Same(argv[i], "-codeonly"))     CodeOnly = true;
		 else if (Same(argv[i], "-static"))  Static = true;
		 else if (Match(argv[i], "-sp3=", Sp3Name))   ;
		 else if (Match(argv[i], "-ecef=", OutputName))    PositionType = ECEF;
		 else if (Match(argv[i], "-enu=", OutputName))     PositionType = ENU;
		 else if (Match(argv[i], "-wgs84=", OutputName))   PositionType = WGS84;
		 else if (Match(argv[i], "-test=", OutputName))    PositionType = TEST;
		 else if (Match(argv[i], "-debug=", arg))          DebugLevel = atoi(arg);
		 else if (Same(argv[i], "-commas"))                OutputType = COMMAS;
		 else if (Same(argv[i], "-simulator"))             Simulator=true;
		 else    return Error("Didn't recognize option %s\n", argv[i]);
	 }

	 if (OutputName == NULL)
		 return Error("Need to specify an output file. (eg. ""-wgs84=file.out"") \n");

	 if (argc-i < 4)
		 return Error("Need to specify: BaseModel BaseFile RovingModel RovingFile\n");

	 BaseModel = argv[i];
	 BasePortName = argv[i+1];
	 RovingModel = argv[i+2];
	 RovingPortName = argv[i+3];

	 return OK;
 }


 bool DisplayOptions()
 {
	 printf("\n");
     printf("Process [options] BaseModel BaseFile RovingModel RovingFile\n");
	 printf("     Double difference postprocessor for GPS data\n");
	 printf("\n");
	 printf("        BaseModel - the type of gps (or data) for the base receiver\n");
	 printf("        BaseFile  - the name of base receiver's data file\n");
	 printf("        RovingModel - type of gps (or data) for the rover\n");
	 printf("        RovingFile - the name of the roving receiver's data file\n");
     printf("\n");
	 printf("    The following ""models"" are supported\n");
	 printf("        RINEX      - Rinex V2.3\n");
	 printf("        XENIR      - Rinex, but with phase reversed\n");
	 printf("        RTCM       - Rtcm104 (RTK) messages xx xx xx\n");
	 printf("        <receiver> - Raw data stream from a gps receiver\n");
	 printf("                     (AC12, ANTARIS, SIRF, LASSENIQ, ALLSTAR, GPS18)\n");
	 printf("\n");    
	 printf("    Where {options} include any of the following:\n");
	 printf("        -static    - the roving receiver is standing still\n");
	 printf("        -codeonly  - do the calculation without carrier phase\n");
     printf("        -sp3=ephfile  - use precise ephemerides from ""file""\n");
	 printf("                     (otherwise, use base receiver's broadcast eph if avail)\n");
	 printf("        -enu=outputfile  - output ENU from Base\n");
	 printf("        -ecef=outputfile - output ECEF (XYZ)\n");
	 printf("        -wgs84=outputfile - output Lat/Lon/Alt (default)\n");
	 printf("        -test=outputfile  - output ENU relative to initial rover position\n");
	 printf("        -commas          - output is comma separated\n");
     printf("    This is version '%s' built on %s %s\n", VERSION, __TIME__, __DATE__);
	 printf("\n");
	 return OK;
 }




static const char* EnuLabels[] = {"Easting", "Northing", "Climbing", "CEP(prototype)", "Fit"};
static const char* EcefLabels[] = {"X", "Y", "Z", "CEP(prototype)", "Fit", NULL};
static const char* Wgs84Labels[] = {"Latitude","Longitude","Altitude(m)","CEP(prototype)","Fit"};
static int MetricPrecision[] = {3, 3, 3, 3, 1};
static int LlaPrecision[] = {8, 8, 3, 3, 1};

static const char** labels[] = {Wgs84Labels, EcefLabels, EnuLabels,EnuLabels};
static int* precisions[] = {LlaPrecision,MetricPrecision,MetricPrecision,MetricPrecision};
static int widths[] = {15, 1};
static const char* separators[] = {" ", ","};



 PositionFormatter::PositionFormatter(const char* OutputFileName, int positiontype, int outputtype)
	 : Formatter(OutputFileName, 5, labels[positiontype], precisions[positiontype],
	             widths[outputtype], separators[outputtype])
 {
 }


 PositionFormatter::~PositionFormatter()
 {
 }





 // Note: we assume the time field will always be first.
 //  Thus, the time field doesn't use a separator, while the others always do.
 //  TODO: turn this into a general purpose formatting class.


 Formatter::Formatter(const char* OutputFileName, int nrfields, const char** label, const int* precision,
	                  int fieldwidth, const char* separator)
	 : OutputFile(OutputFileName), NrFields(nrfields), FieldWidth(fieldwidth), Separator(separator)
 {
	 if (ErrCode != OK) return;
  
	 // Remember the precision of the numbers
	 for (int i=0; i<NrFields; i++)
		 Precision[i]  = precision[i];

	 // Output the Labels
	 ErrCode = Header(label);

	 return;
 }

 bool Formatter::Data(double p0, ...)
 {
	 va_list args;
	 va_start (args, p0);

	// Display each number
     Numeric(p0, FieldWidth, Precision[0]);
	 for (int i=1; i<NrFields; i++)
		Numeric(va_arg(args, double), FieldWidth, Precision[i]);

	// End of line
	WriteLine("");

	va_end(args);
	return OK;
 }


 bool Formatter::Header(const char** label)
 {
	 // Output a label for Time  (HACK! want it centered if space separated.)
	 if (Separator[0] == ' ')   Printf("     Date-Time     ");
	 else                       Printf("Date-Time");

	 // output each label
	 for (int i=0; i<NrFields; i++)
	    if (Centered(label[i], FieldWidth) != OK) return Error();

	 if (WriteLine("") != OK) return Error();

	 return OK;
 }
	

 bool Formatter::Centered(const char* field, int width)
 {

    // decide how many leading spaces to output (at least one leading)
	int len = strlen(field);
	int leading = max(0, (width-len+1)/2);
	int trailing = max(0, width-len-leading);

	// output the field with it's separator
	return Printf("%s%*s%*s", Separator, leading+len, field, trailing, "");
 }

 bool Formatter::Numeric(double value, int width, int precision)
 {
	 return Printf("%s%*.*f", Separator, width, precision, value);
 }



 Formatter::~Formatter()
 {
 }

