#ifndef MTI_DATA_GENERATOR_H
#define MTI_DATA_GENERATOR_H

#include "MTIDefinitions.h"

#define _USE_MATH_DEFINES
#ifdef _MSC_VER
	#define _CRT_SECURE_NO_WARNINGS
#endif
#define MTI_MAX_NUM_FRAMES			32
#define DATAFORMAT 0

#define USBSLMZ
#ifdef USBSLM4
	#define MTI_MAX_NUM_SAMPLES			25000
#endif
#ifdef USBSLMZ
	#define MTI_MAX_NUM_SAMPLES			100000
#endif
#ifdef MTI_ANDROID
	#define DLLEXPORT
#endif
///////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <time.h>
#include "DspFilter.h"

#ifdef _MTIOPENCV
	#include <cv.h>
	#include <highgui.h>
#endif

///////////////////////////////////////////////////////////////////////////////////
//MTIDataGenerator encapsulates the content generation API
class DLLEXPORT MTIDataGenerator
{
private:
	// Transforms
	float			m_fPMat[ 3 ][ 3 ], m_fBarMult[ 101 ];
	bool			m_bProjXfmInv, m_bBarXfmInv;
	// ILDA
	unsigned int	m_iIldaNumFrames, m_iIldaFrameType, *m_iIldaFrameIndex, *m_iIldaFrameIds, *m_iIldaNumKpt, *m_iIldaNumSmp, m_iIldaNumMaxPoints, m_iIldaNumTotalPoints;
	short			*m_iXIlda, *m_iYIlda, *m_iZIlda, *m_iMIlda, *m_iRIlda, *m_iGIlda, *m_iBIlda;
	// Hershey
	unsigned int	m_iHersheyNumMath, m_iHersheyNumSym, m_iHersheyNumFonts, m_iHersheyNumCharsOC, m_iHersheyNumCharsOR;
	short			*m_iHersheyMapMath, *m_iHersheyMapSym, **m_iHersheyMapFonts, **m_iHersheyDataOC, **m_iHersheyDataOR;
	// Point File
	float			*m_fXFile, *m_fYFile, *m_fTFile;
	unsigned char	*m_iMFile;
	unsigned int	m_iNumFilePoints, m_iPointFileSps, m_iPointTimeFileSps;
	// Filter
	Dsp::Filter     *m_filter;
	double			m_fIIRFilterA[ DSPFILTERS_IIRGENERIC_MAXORDER ], m_fIIRFilterB[ DSPFILTERS_IIRGENERIC_MAXORDER ];
	unsigned int    m_iIIRFilterOrder, m_iIIRFilterSps;
	// Interp Data
	float			*m_fDs;
	unsigned int	m_iMaxKeyPoints, m_iKeyOnLevel;

	unsigned int CopyKeypointsToSamples(float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample, int numKeyPointsToCopy);

public:

	/** @name Creation and Termination
	*  Constructor and Destructor of the MTIDataGenerator class
	*/
	//@{
	MTIDataGenerator();
	~MTIDataGenerator();
	//@}

	/** @name Transforms
	*  Transform data for content generation
	*/
	//@{
	void BoundingBox( float* xData, float* yData, unsigned int numPoints, float& xMin, float& xMax, float& yMin, float& yMax );
	unsigned int NormalizeData( float* xData, float* yData, unsigned int numPoints, float amplitude, float theta, float& xMin, float& xMax, float& yMin, float& yMax ); 
	unsigned int AffineTransformData( float* xData, float* yData, unsigned int numPoints, float amplitude, float theta, float xof, float yof );
	void InitProjectiveTransformMatrix( float* x, float* y, bool inverse );
	unsigned int ProjectiveTransformData( float* xData, float* yData, unsigned int numPoints );
	unsigned int TangentialTransformData( float* xData, float* yData, unsigned int numPoints, float thetaMax );
	void InitRadialTransformMult( bool inverse );
	unsigned int RadialTransformData( float* xData, float* yData, unsigned int numPoints, float k );
	unsigned int TransformData( float* xData, float* yData, unsigned int numPoints, float amplitude, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK );
	//@}

	/** @name Utilities
	*  Utilities for content generation
	*/
	//@{
	int gcd( int a, int b );
	float saw( float theta );
	unsigned int AnimationRotate( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int numFrames );
	unsigned int AnimationRotate( float* xData, float* yData, unsigned char* mData, unsigned int* rgbData, unsigned int numPoints, unsigned int numFrames );
	unsigned int AnimationScroll( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int numFrames );
	unsigned int AnimationScroll( float* xData, float* yData, unsigned char* mData, unsigned int* rgbData, unsigned int numPoints, unsigned int numFrames );
	unsigned int AnimationBounce( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int numFrames, float theta );
	unsigned int AnimationBounce( float* xData, float* yData, unsigned char* mData, unsigned int* rgbData, unsigned int numPoints, unsigned int numFrames, float theta );
	unsigned int CloseCurve( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int closePoints, bool retrace = false );
	unsigned int CloseCurve( float* xData, float* yData, unsigned char* mData, unsigned int* rgbData, unsigned int numPoints, unsigned int closePoints, bool retrace = false );
	//@}

	/** @name Waveforms
	*  Useful waveforms for content generation
	*/
	//@{
	unsigned int SineWaveform( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, float amplitude, unsigned int frequency, float phase );
	unsigned int SawtoothWaveform( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, float amplitude, unsigned int frequency, float width, float phase );
	unsigned int SquareWaveform( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, float amplitude, unsigned int frequency, float width, float phase );
	unsigned int NoiseWaveform( float* xData, unsigned int numPoints, float amplitude );
	unsigned int RQWaveform(float* xData, float* yData, unsigned char* mData, unsigned int& sps, bool sawtoothOnY, float xAmplitude, float yAmplitude, unsigned int xFrequency,
		float numPeriods, float dutyCycle, float yBandwidth);
	//@}

	/** @name Curves
	*  Curves for content generation
	*/
	//@{
	unsigned int SpiralCurve( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, float xAmplitude, float yAmplitude, unsigned int frequency, float phase );
	unsigned int PolygonCurve( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, float amplitude, unsigned int frequency, float phase );
	unsigned int SpirographCurve( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int curveType, unsigned int waveType, float amplitude, int p1, int p2, int p3 );
	unsigned int LissajousCurve( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int waveType, float amplitude, unsigned int xFreq, unsigned int yFreq, float phase,	unsigned int modType, float modAmplitude, unsigned int modFreq, float modPhase );
	unsigned int AnalogClockCurve( float* xData, float* yData, unsigned char* mData, unsigned int hrs, unsigned int min, unsigned int sec, float amplitude );
	unsigned int DigitalClockCurve( float* xData, float* yData, unsigned char* mData, unsigned int hrs, unsigned int min, unsigned int sec, unsigned int fontIndex, float amplitude, float theta, bool retrace );
	unsigned int DynamicsCurve( float* xData, float* yData, unsigned char* mData, unsigned int numPoints, float x0, float y0, float xVel, float yVel, float xAcc, float yAcc, float fRatio );
	//@}

	/** @name ILDA Files
	*  Tools to load and display ILDA files
	*/
	//@{
	void ClearIldaData();
	void LoadIldaFile( const char* filename );
	unsigned int IldaFrameType()	{ return m_iIldaFrameType; };
	unsigned int IldaNumFrames()	{ return m_iIldaNumFrames; };
	unsigned int IldaNumMaxPoints()	{ return m_iIldaNumMaxPoints; };
	unsigned int IldaNumTotalPoints()	{ return m_iIldaNumTotalPoints; };
	unsigned int IldaFrameSize( unsigned int frameId );
	unsigned int IldaAnimSize( unsigned int* frameIds, unsigned int numFrames );
	unsigned int IldaDataStream( float* xData, float* yData, unsigned char* mData, unsigned int* frameIds, unsigned int numFrames, float amplitude, float theta, float theta1, float theta2, bool retrace );
	unsigned int IldaDataStream( float* xData, float* yData, unsigned char* mData, unsigned int* rgbData, unsigned int* frameIds, unsigned int numFrames, float amplitude, float theta, float theta1, float theta2, bool retrace );
	//@}

	/** @name Hershey Text
	*  Tools to generate Hershey text
	*/
	//@{
	void ClearHersheyData();
	void LoadHersheyFile( const char* filename );
	unsigned int HersheyNumMath()	{ return m_iHersheyNumMath; };
	unsigned int HersheyNumSym()		{ return m_iHersheyNumSym; };
	unsigned int HersheyNumOR()		{ return m_iHersheyNumCharsOR; };
	unsigned int HersheyNumOC()		{ return m_iHersheyNumCharsOC; };
	unsigned int HersheyIndexMath( unsigned int i )	{ return ( m_iHersheyMapMath ) ? m_iHersheyMapMath[ i ] : -1; };
	unsigned int HersheyIndexSym( unsigned int i )	{ return ( m_iHersheyMapSym ) ? m_iHersheyMapSym[ i ] : -1; };
	unsigned int HersheyIndexFont( unsigned int i, unsigned int j )	{ return ( m_iHersheyMapFonts ) ? m_iHersheyMapFonts[ i ][ j ] : -1; };
	unsigned int HersheyDataSize( unsigned int* frameIds, unsigned int numFrames, bool ocMode );
	unsigned int HersheyDataSize( const char* text, unsigned int numChars, unsigned int fontIndex );
	unsigned int HersheyDataStream( float* xData, float* yData, unsigned char* mData, unsigned int* charIds, unsigned int numChars, bool ocMode, float amplitude, float theta, bool retrace );
	unsigned int HersheyDataStream( float* xData, float* yData, unsigned char* mData, const char* text, unsigned int numChars, unsigned int fontIndex, float amplitude, float theta, bool retrace );
	//@}


	/** @name Point Files
	*  Point files have 3 columns, for X data (-1 to 1), Y data (-1 to 1), and M data (0-255 for digital output)
	*  File may specify sample per second rate SPS in the first line
	*/
	//@{
	void ClearPointFileData();
	void AllocPointFileData( unsigned int numPoints );
	unsigned int LoadPointFile( const char* filename );
	unsigned int PointFileDataStream( float* xData, float* yData, unsigned char* mData );
	unsigned int GetPointFileSize() { return m_iNumFilePoints; };
	unsigned int GetPointFileSps() { return m_iPointFileSps; };
	void ExportFile( const char* filename, float* xData, float* yData, unsigned char* mData, unsigned int numPoints, unsigned int sps );
	static float ReadFileParameter( const char* filename, const char *paramName, float paramValue = 0.f );
	//@}

	/** @name Point-Time Files - Files
	*  Point-Time files have 4 columns, for X data (-1 to 1 float), Y data (-1 to 1 float), M data (0-255 for digital output), and T data (0 to 1 float) for time duration at specified XYM values
	*  File may specify sample per second rate SPS in the first line
	*/
	//@{
	void ClearPointTimeFileData();
	void AllocPointTimeFileData( unsigned int numPoints );
	unsigned int LoadPointTimeFile( const char* filename );
	unsigned int PointTimeFileDataStream( float* xData, float* yData, unsigned char* mData, float* tData );
	unsigned int GetPointTimeFileSize() { return m_iNumFilePoints; };
	unsigned int GetPointTimeFileSps() { return m_iPointTimeFileSps; };
	//@}

	/** @name Interpolation
	*  Functions for interpolating data using various interpolation methodologies
	*/
	//@{
	void ClearInterpData();
	void AllocInterpData( unsigned int numKeyPoints, unsigned int numSamplePoints );
	unsigned int InterpolateData( float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample, unsigned int numKeyPoints, unsigned int numSamplePoints, float ooRatio = 2.5, float onFrac = 0.1, float offFrac = 0.5 );
	unsigned int InterpolateDataLinear( float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample, unsigned int numKeyPoints, unsigned int numSamplePoints, float ooRatio = 2.5 );
	unsigned int InterpolateDataTrapezoidal( float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample, unsigned int numKeyPoints, unsigned int numSamplePoints, float ooRatio = 2.5, float onFrac = 0.1, float offFrac = 0.5 );
	unsigned int InterpolateDataOptimized(  float* xKey, float* yKey, unsigned char* mKey, unsigned int* rgbKey, float* xSample, float* ySample, unsigned char* mSample, unsigned int* rgbSample, unsigned int numKeyPoints, unsigned int numSamplePoints );
	unsigned int InterpolateDataOptimized(  float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample, unsigned int numKeyPoints, unsigned int numSamplePoints );
	unsigned int OptimizeKeypoints(  float* xKey, float* yKey, unsigned char* mKey, unsigned int numKeyPoints, unsigned int targetNumKeyPoints = 0 );
	unsigned int OptimizeKeypoints(  float* xKey, float* yKey, unsigned char* mKey, unsigned int* rgbKey, unsigned int numKeyPoints, unsigned int targetNumKeyPoints = 0 );
	//@}

	/** @name Filtering
	*  Functions for filtering data
	*/
	//@{
	void LoadIIRFilterParams( const char* filename );
	unsigned int GetIIRFilterOrder() { return m_iIIRFilterOrder; };
	unsigned int GetIIRFilterSps() { return m_iIIRFilterSps; };
	void SetupSoftwareFilter( unsigned int type, unsigned int order, float cutoffFreq, float sampleFreq, bool twoChannel = true );
	void FilterData( float* xData, float* yData, float *xFilt, float *yFilt, unsigned int numPoints, bool zeroPhase = true );
	void FilterData( float* xData, float *xFilt, unsigned int numPoints, bool zeroPhase = true );
	//@}

	/** @name Scan Patterns
	*  Functions to generate various scan patterns
	*/
	//@{

	//! LinearRasterPattern constructs a linear raster scan pattern based on the given parameters.
	//!
	//!	Returns the number of points in the constructed scan pattern
	//!
	//! @param[in,out]	xData	X position output container with pre-allocated length (recommended 100,000 points)
	//! @param[in,out]	yData	Y position output container with pre-allocated length (recommended 100,000 points)
	//! @param[in,out]	mData	8-bit Digital Output number output container with pre-allocated length (recommended 100,000 points)
	//!		Bit 0: On during the "active" portion of line
	//!		Bit 1: On during the pixel portion of line
	//!		Bit 2: Pulsed at the beginning of the frame
	//!		Bit 3: Indicated Y-Axis direction
	//!
	//! @param[in]	xAmp	Amplitude of X-axis of the raster scan (normalized from 0.0 to 1.0)
	//! @param[in]	yAmp	Amplitude of Y-axis of the raster scan (normalized from 0.0 to 1.0)
	//!
	//! @param[in]	numLines	Total number of raster lines. During each line scan there is a high output on DOut0, and
	//!		during each turn-around there is a low output on DOut0.
	//!
	//!	@param[in]	numPixels	When ppMode = true, numPixels corresponds to the number of stop-and-go locations ("pixels")
	//!		along a line. In both ppMode = false and ppMode = true, numPixels corresponds to the number of equally spaced
	//!		Digital Output pulses along a line, contained in Bit 1 of mData.
	//!
	//! @param[in]	lineTime	Duration of time allocated to each raster line in seconds. The time is specifically allocated
	//!		to the 'active' portion of each line. The actual scan along one line will include a small additional percentage of
	//!		time for turn-around.
	//! 
	//! @param[in]	ppMode	ppMode = 1 corresponds to point-to-point raster with near stopping at each point. ppMode = 0 corresponds
	//!		to a uniform velocity line raster.
	//!
	//!	@param[in]	retrace	Toggles between bi-directional writing on (1) and off (0)
	//!
	//! @param[in]	triggerShift	Number of samples the synchronous digital output should be rotated with respect to the MEMS
	//!		Driver X, Y output (can be negative or positive). User may want to differently align the locations of the TTL pulses on
	//!		DOut0-3 and can therefore rotate forward or backward a number of samples.
	//!
	//! @param[in]	theta	Rotation angle for the raster pattern (looking toward display surface) in radians. Passing 0 results in vertical lines.
	//!
	//! @param[in, out]	sps	Sample rate of raster scan. The user can pre-set this sample rate, however the function will likely override
	//!		and return a rate based on the numLines and numPixels input to best match the desired scan
	//!
	//!	@param[in]	spsMin	The MEMS Controller's min sample rate used in ensuring the function does not output a sample rate less than the minimum
	//!	@param[in]	spsMax	The MEMS Controller's max sample rate used in ensuring the function does not output a sample rate more than the maximum
	unsigned int LinearRasterPattern( float* xData, float* yData, unsigned char* mData, float xAmp, float yAmp, unsigned int numLines, unsigned int numPixels, float lineTime, bool ppMode, bool retrace, int triggerShift, float theta, unsigned int& sps, int spsMin, int spsMax );

	unsigned int LinearRasterPatternImaging( float* xData, float* yData, unsigned char* mData, float xAmp, float yAmp, unsigned int numLines, unsigned int numPixels, float lineTime, bool ppMode, bool retrace, int triggerShift, float theta, unsigned int& sps, int spsMin, int spsMax );
	unsigned int PointToPointPattern( float* xData, float* yData, unsigned char* mData, float* xPoints, float* yPoints, unsigned char* mPoints, float* tPoints, int nSamples, float stepTime, unsigned int& sps, int spsMin, int spsMax );
	void LaserWriterInterpData( int inCount, int outCount, unsigned char* inM, float* inX, float* inY,unsigned char* outM, float* outX, float* outY, float DON, float DOFF, int closed_trajectory );
	void LaserWriterInterpData1Seg( int inCount, int outCount, float* inX, float* outX, float DON );
#ifdef _MTIOPENCV
	unsigned int LaserWriterDataStream( float* x, float* y, unsigned char* m, unsigned char* stream, int* lastOnPixelList, unsigned int lines, unsigned int numPixels, float lineTime, bool compressLine, unsigned int& sps, int spsMin, int spsMax, float xMin, float xMax, float yMin, float yMax );
	unsigned int ImageRasterPattern( float* xData, float* yData, unsigned char* mData, float xAmplitude, float yAmplitude, unsigned int numLines, unsigned int numPixels, IplImage* srcImage, bool bwMode, int bwThreshold, float lineTime, bool compressLine, unsigned int& sps, int spsMin, int spsMax );
#endif
	//@}

	/** @name MirrorcleDraw Wrappers
	*  MirrorcleDraw Wrappers
	*/
	//@{
	unsigned int CurvesDataSize( unsigned int curveType, unsigned int sps, float rr, unsigned int animType, float atime, bool retrace, unsigned int fileType, unsigned int m1, unsigned int m2, float p2, float p3, float p4, const char* txt, unsigned int numChars );
	unsigned int GenerateAnimations( float* xSample, float* ySample, unsigned char* mSample, unsigned int animType, unsigned int numSamplePoints, unsigned int numAnimFrames );
	unsigned int GenerateAnimations( float* xSample, float* ySample, unsigned char* mSample, unsigned int* rgbSample, unsigned int animType, unsigned int numSamplePoints, unsigned int numAnimFrames );
	void GenerateWaveformData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int m1, float p1, float p2, float p3, float p4, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateSpirographData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType,
		unsigned int m1, unsigned int m2, float p1, float p2, float p3, float p4, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateLissajousData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int m1, unsigned int m2, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK 
	);
	void GenerateImportFileData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool interpILDA , bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int fileType, float Amplitude, float p2, float p3, float p4, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateImportFileData(
		float* xKey, float* yKey, unsigned char* mKey, unsigned int* rgbKey, float* xSample, float* ySample, unsigned char* mSample, unsigned int* rgbSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool interpILDA , bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int fileType, float Amplitude, float p2, float p3, float p4, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateTextData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int TextMode, unsigned int TextFont, float Amplitude, float CharID, const char* txt, unsigned int numChars, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateScanData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int& sps, float rr, bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int m1, unsigned int m2, float p1, float p2, float p3, float p4, float e1, float e2, unsigned int bwc, void* pImage, unsigned int spsMin, unsigned int spsMax, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateClockData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int sps, float rr, bool retrace, float oor, float onFrac, unsigned int animType, float atime, unsigned int filtType, 
		unsigned int hrs, unsigned int min, unsigned int sec, unsigned int ClockType, unsigned int TextFont, float Amplitude, float theta, float xof, float yof, bool projXfm, float tanXfmTheta, float barXfmK
	);
	void GenerateSignalData(
		float* xKey, float* yKey, unsigned char* mKey, float* xSample, float* ySample, unsigned char* mSample,
		unsigned int& numKeyPoints, unsigned int& numSamplePoints, unsigned int& sps, unsigned int sigType, unsigned int m1, unsigned int m2, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float xof, float yof
	);
	//@}
};

#endif