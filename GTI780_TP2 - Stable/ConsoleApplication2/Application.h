#pragma once
#define _CRT_SECURE_NO_DEPRECATE
#include "Application.h"
#include "stdafx.h"
#include <Windows.h>
#include <opencv2\highgui\highgui.hpp>
#include <Kinect.h>

#define USHRT_MAX     0xffff        /* maximum unsigned short value */

class Application
{
	// Change these values to modify the placement of the openCV window.
	static const unsigned int	cWindowPosX = 0;
	static const unsigned int	cWindowPosY = 0;
	static const unsigned int	cWindowWidth = 1280;
	static const unsigned int	cWindowHeight = 720;

	static const int        cDepthWidth  = 512;
    static const int        cDepthHeight = 424;
    static const int        cColorWidth  = 1920;
    static const int        cColorHeight = 1080;
	static const int		cMapDepthToByte = 8000 / 256;
	static const unsigned int	cColorBufferSize = cColorWidth * cColorHeight * 4 * sizeof( unsigned char );
	
	// Add some constants for the different parameters for depth to disparity conversion.

public:
								Application(void);
								~Application(void);
	
	int							Run(void);

private:

	// Kinect stuff
    IKinectSensor*				m_pKinectSensor;				// Kinect Sensor
    IMultiSourceFrameReader*	m_pMultiSourceFrameReader;		// Multi source frame reader
    ICoordinateMapper*			m_pCoordinateMapper;			// Coordinate mapper
    DepthSpacePoint*			m_pDepthCoordinates;			// Mapped Points
	USHORT						m_DepthMinReliableDistance;		// Minimum depth reliable distance
	BYTE*						m_pLUTDepth2Display;			// Lookup table to convert depth 16 to depth 8 bytes for display
	int*						m_pLUTD2D;						// Lookup table to convert depth to disparity


	
	// This is where you want to add your filtering of the depth map.
	cv::Mat						ApplyFiltering(cv::Mat src);

	// This is where you want to add your code to display images side by side
	void						DisplaySideBySide(cv::Mat left, cv::Mat right);

	// This is where to add your code for pixel shift
	cv::Mat						DIBRRIGHT(cv::Mat left, cv::Mat depthMap, int* LUTD2D);

	// This is where you build the Lookup table for depth -> disparity
	int*						LUTDepthToDisparity();

	// This is to speed up the display of the depth
	BYTE*						LUTDepthToDisplay(int depthMinReliableDistance, int depthMaxReliableDistance);

	// Process the depth and color frames
	void						ProcessFrame(cv::Mat colorMat, int nColorWidth, int nColorHeight, 
												UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, 
												int nDepthMaxReliableDistance, int nDepthMinReliableDistance);
	
	// Initializes the default Kinect sensor
	HRESULT						InitializeDefaultSensor();

	// Initialize the main CV window
	void						InitializeDefaultWindow();

	// Main processing function
	void						Update();

};

