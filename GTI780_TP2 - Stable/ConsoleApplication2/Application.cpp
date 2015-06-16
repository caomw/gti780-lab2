#include "Application.h"
#include "stdafx.h"
#include <Windows.h>
#include <opencv2\highgui\highgui.hpp>
#include <Kinect.h>
#include <cv.h>


Application::Application(void) :
	m_pKinectSensor(NULL),
	m_pMultiSourceFrameReader(NULL),
	m_pCoordinateMapper(NULL),
	m_pDepthCoordinates(NULL),
	m_pLUTD2D(NULL),
	m_DepthMinReliableDistance(0),
	m_pLUTDepth2Display(NULL)
{
	// create heap storage for the coorinate mapping from color to depth
    m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];

	float H = 0.80; // mètres
	float W = 1.33;	// mètres
	int N = 8;	// bits
	float knear = 0.1;
	float kfar = 0.2;
	float tc = 0.065;	// mètres
	float D = 3 * H;	// mètres
	// Create the LUT for depth to disparity
	// Don't forget to pass the correct values here
	m_pLUTD2D = LUTDepthToDisparity(H, W, N, knear, kfar, tc, D);
}


Application::~Application(void)
{
	SafeRelease(m_pMultiSourceFrameReader);

	SafeRelease(m_pCoordinateMapper);

	if (m_pKinectSensor) m_pKinectSensor->Close();
	SafeRelease(m_pKinectSensor);

	if (m_pDepthCoordinates)
    {
        delete[] m_pDepthCoordinates;
        m_pDepthCoordinates = NULL;
    }

	if(m_pLUTD2D)
	{
		delete[] m_pLUTD2D;
		m_pLUTD2D = NULL;
	}

	if(m_pLUTDepth2Display)
	{
		delete[] m_pLUTDepth2Display;
		m_pLUTDepth2Display = NULL;
	}
}

int Application::Run(void)
{
	HRESULT hr;
	
    hr = InitializeDefaultSensor();	// Get and initialize the default Kinect sensor

	if(SUCCEEDED(hr)) InitializeDefaultWindow(); // Initialize the default OpenCV Window

	if(!SUCCEEDED(hr)) return -1;

	// Main loop to receive images from kinect
	while(true)
	{
		// Update until the Escape key has been pressed
		Update();

		if( cv::waitKey( 30 ) == VK_ESCAPE )
		{
			break;
		}
	}

	return 0;
}

// This is the main Kinect function that is called every loop. 
// This function acquires the frames, and calls the process on the depth
void Application::Update()
{
	if(!m_pMultiSourceFrameReader) return;	// If the reader is not available, return

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);	// Acquire a multi frame

	// ======================== Acquire Depth Frame ===========================
	if(SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference); // Acquire a depth frame reference
		if(SUCCEEDED(hr)) hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);	// Acquire a depth frame
		SafeRelease(pDepthFrameReference);	// Release the depth frame reference now that we have the depth frame
	}

	// ========================= Acquire Color Frame ===========================
	if(SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);	// Acquire a color frame reference
		if(SUCCEEDED(hr)) hr = pColorFrameReference->AcquireFrame(&pColorFrame);	// Acquire a colo frame
		SafeRelease(pColorFrameReference);	// Release the color frame reference now that we have the color frame
	}

	// If everything went well until this point we can start using the frames
	if(SUCCEEDED(hr))
	{
		
		// =============================== COLOR =============================
        IFrameDescription* pColorFrameDescription = NULL;
        int nColorWidth = 0;
        int nColorHeight = 0;
        ColorImageFormat imageFormat = ColorImageFormat_None;

		// Get color frame data
		if(SUCCEEDED(hr)) hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);	// Get the color frame description
		if(SUCCEEDED(hr)) hr = pColorFrameDescription->get_Width(&nColorWidth);	// Get the width of the color frame
		if(SUCCEEDED(hr)) hr = pColorFrameDescription->get_Height(&nColorHeight);	// Get the height of the color frame
		if(SUCCEEDED(hr)) hr = pColorFrame->get_RawColorImageFormat(&imageFormat);	// Get the image format of the color frame

		cv::Mat colorMat( cColorHeight, cColorWidth, CV_8UC4 );	// Create an openCV matrix to contain the color frame Unsigned 8 bytes per pixels, 4 channels

		// Convert color frame to a CV matrix
		hr = pColorFrame->CopyConvertedFrameDataToArray(cColorBufferSize, reinterpret_cast<BYTE*>( colorMat.data ), ColorImageFormat_Bgra); // Copy the color frame into the matrix
		SafeRelease(pColorFrame);	// Release the color frame
		SafeRelease(pColorFrameDescription);	// Release the color frame description

		// =============================== DEPTH ==============================
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *pDepthBuffer = NULL;
		USHORT nDepthMaxReliableDistance = 0;
		USHORT nDepthMinReliableDistance = 0;

		// Get depth frame data
		if(SUCCEEDED(hr)) hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);	// Get the depth frame description
		if(SUCCEEDED(hr)) hr = pDepthFrameDescription->get_Width(&nDepthWidth);	// Get the depth width
		if(SUCCEEDED(hr)) hr = pDepthFrameDescription->get_Height(&nDepthHeight);	// Get the depth height
		if(SUCCEEDED(hr)) hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);	// Get the max depth reliable distance
		if(SUCCEEDED(hr)) hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);	// Get the min depth reliable distance

		// If the reliable distance has changed, re-create the LUT for depth to display.
		if(nDepthMinReliableDistance != m_DepthMinReliableDistance)
		{
			m_DepthMinReliableDistance = nDepthMinReliableDistance;
			m_pLUTDepth2Display = LUTDepthToDisplay(m_DepthMinReliableDistance, USHRT_MAX);	// Create a lookup table to convert depth to display depth
		}

		if(SUCCEEDED(hr)) hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);	// Access directly the depth buffer
		
		SafeRelease(pDepthFrameDescription);	// Release the depth frame description

		// Process the two images and display
		if(SUCCEEDED(hr)) ProcessFrame(colorMat, nColorWidth, nColorHeight, pDepthBuffer, nDepthWidth, nDepthHeight, nDepthMaxReliableDistance, nDepthMinReliableDistance);
	}

	SafeRelease(pDepthFrame);	// Release the depth frame now that we are done with it
    SafeRelease(pMultiSourceFrame);	// Release the multi frame now that we are done with it
}

// Process the depth frame to map it to the color image and display
void Application::ProcessFrame(cv::Mat colorMat, int nColorWidth, int nColorHeight, 
							   UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight, 
							   int nDepthMaxReliableDistance, int nDepthMinReliableDistance)
{
	// Verify that the data we have is in the format we want it to be
	if((nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pDepthBuffer && m_pDepthCoordinates && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight))
	{

		// =============================== DEPTH ==============================

		// Map the color frame into the depth space
		HRESULT hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, 
																	(UINT16*)pDepthBuffer, nColorWidth * nColorHeight, 
																	m_pDepthCoordinates);

		if(SUCCEEDED(hr))
		{
			cv::Mat depthMat = cv::Mat::zeros(cColorHeight, cColorWidth, CV_8UC3);	// Create the depth map to store the mapped depth.

			for(int colorIndex = 0; colorIndex < (nColorWidth * nColorHeight); ++colorIndex)
			{
				DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())	// A value of -Inf means a bad mapping
				{
					int depthX = static_cast<int>(p.X + 0.5f);	// Round the coordinate to closest int
					int depthY = static_cast<int>(p.Y + 0.5f);	// Round the coordinate to closest int

					if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))	// If the depth is inside the image
					{
						UINT16 depth = pDepthBuffer[depthX + (depthY * cDepthWidth)];	// Get the corresponding depth to the point

						BYTE intensity = m_pLUTDepth2Display[depth];	// Use the lookup table to convert the depth into a visual representation

						// The image has 3 color channels so we need to write the intensity in all channels
						depthMat.data[colorIndex*3] = intensity;
						depthMat.data[colorIndex*3 +1] = intensity;
						depthMat.data[colorIndex*3 +2] = intensity;
					}
				}
			}

			// Most of OpenCV filtering uses BGR images instead of BGRA which is the format of the output of the kinect
			// We also want to crop the images to get rid of the left/right borders in the depth that is all black due to the mapping.
			cv::Rect mROI(200, 0, cColorWidth-401, cColorHeight);	// The border of 200 pixels that we remove has been calculated by hand

			cv::Mat depthConvert;
			cv::cvtColor(depthMat, depthConvert, CV_BGRA2BGR);
			depthConvert = depthConvert(mROI);

			cv::Mat colorConvert;
			cv::cvtColor(colorMat, colorConvert, CV_BGRA2BGR);
			colorConvert = colorConvert(mROI);

			// ============================= Filter your depth image here =================================
			cv::Mat depthResult = ApplyFiltering(depthConvert);	// Apply your filtering on the depth map here and store the result

			// ============================= Apply DIBR processing here ===================================
			cv::Mat mRight = DIBRRIGHT(colorConvert, depthResult, m_pLUTD2D);	// Apply your DIBR here and store the result as the right image

			// ============================= Display your images here =====================================
			DisplaySideBySide(colorConvert, mRight);	// Display the images side by side
		}
	}
}

// Apply the filtering to your depth map
cv::Mat Application::ApplyFiltering(cv::Mat src)
{
	//Filtre Gaussien
	cv::Mat dst(src.rows, src.cols, CV_8UC1);	//conversion en 8bits unsigned 1 channel
	cv::cvtColor(src, dst, cv::COLOR_RGB2GRAY);	//conversion en tons de gris

	uchar fillValue = 255;

	cv::dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
	//cv::erode(src, dst, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::GaussianBlur(dst, dst, cv::Size(15, 15), 5, 5);

	cv::Mat image_thresh(dst.rows, dst.cols, CV_8UC1);
	// Tout ce qui est plus que grand que 0 (tout ce qui n'est pas noir) est égal à 0, sinon = 255
	cv::threshold(dst, image_thresh, 0, 255, cv::THRESH_BINARY);
	cv::Mat mask;
	image_thresh.copyTo(mask);

	// Belle bordure
	cv::copyMakeBorder(mask, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);
	//cv::floodFill(dst, mask, seed, cv::Scalar(255, 255, 0), 0, cv::Scalar(0), cv::Scalar(0), 8 | (fillValue << 8));

	// inverser mask
	// while
	// find non zeros
	// inverser mask (pour floodfiller des espaces noirs, blancs impossible)
	// floodfill : cv::floodFill(dst, mask, seed, cv::Scalar(255, 255, 0), 0, cv::Scalar(0), cv::Scalar(0), 8 | (fillValue << 8));
	// inverser mask
	// find non zeros

	cv::bitwise_not(mask, mask);	//noir
	int n = 0;
	//std::vector<cv::Point2i> nombre_non_zero;
	cv::Rect roi = cv::Rect(1, 1, mask.cols - 2, mask.rows - 2);
	cv::Mat nombre_non_zero;
	cv::findNonZero(mask(roi), nombre_non_zero);	// trouve les tâches blanches, blanc
	n = cv::countNonZero(mask(roi));

	while (n > 0) {
		cv::Point seed(nombre_non_zero.at<cv::Point>(0));
		//cv::Point seed(nombre_non_zero[0].x, nombre_non_zero[0].y);
		cv::bitwise_not(mask, mask);	//noir
		// trouver la couleur maximale (la plus pâle qui n'est pas 0)
		int buffer = 15;
		cv::Mat rect;

		cv::bitwise_not(mask, mask);
		if ((seed.x - (buffer / 2) > 0) && (seed.x + (buffer / 2) < mask(roi).cols) && (seed.y - (buffer / 2) > 0) && (seed.y + (buffer / 2) < mask(roi).rows)) {
			cv::Rect rect_min_color = cv::Rect(seed.x - (buffer / 2), seed.y - (buffer / 2), buffer, buffer);
			cv::findNonZero(mask(rect_min_color), rect);
		}
		else {

			int buffer_y = buffer;
			int buffer_x = buffer;

			if ((mask(roi).cols - seed.x) < buffer) {
				buffer_x = (mask(roi).cols - seed.x);
			}

			if ((mask(roi).rows - seed.y) < buffer) {
				buffer_y = (mask(roi).rows - seed.y);
			}

			cv::Rect rect_min_color = cv::Rect(seed.x, seed.y, buffer_x, buffer_y);
			cv::findNonZero(mask(rect_min_color), rect);
		}	
		cv::bitwise_not(mask, mask);
			
		cv::Point seed_min(rect.at<cv::Point>(0));

		uchar min_color = mask.at<uchar>(seed_min.x, seed_min.y);

		cv::floodFill(dst, mask, seed, cv::Scalar(30), 0, cv::Scalar(0), cv::Scalar(30), 8 | (fillValue << 8));
		cv::bitwise_not(mask, mask);	//blanc
		//nombre_non_zero.clear();
		cv::findNonZero(mask(roi), nombre_non_zero);	// trouve les tâches blanches, blanc
		n = cv::countNonZero(mask(roi));
	}
	
	cv::cvtColor(dst, dst, cv::COLOR_GRAY2RGB);
	return dst;
}

// Display two openCV Images side by side
void Application::DisplaySideBySide(cv::Mat left, cv::Mat right)
{
	cv::Mat catLeftRight; // Change this so your images left and right are displayed side by side
	
	// Display your images
	cv::hconcat(left, right, catLeftRight);
	cv::imshow("GTI780_TP2", catLeftRight);
}

// Apply the DIBR algorithm here using the Lookup table.
cv::Mat	Application::DIBRRIGHT(cv::Mat left, cv::Mat depthMap, int* LUTD2D)
{
	cv::Mat mRight = cv::Mat::zeros(left.rows, left.cols, CV_8UC3); // Change this so your mRight correspond to left+pixelShift

	for (int x = 0; x < depthMap.rows - 1; x++) {
		for (int y = 0; y < depthMap.cols - 1; y++) {
			uchar depth = depthMap.at<uchar>(x, y);

			if (((y + LUTD2D[depth]) > depthMap.cols - 1) || ((y + LUTD2D[depth]) < 0)) {
				mRight.at<cv::Vec3b>(x, y) = left.at<cv::Vec3b>(x, y);
			}
			else {
				mRight.at<cv::Vec3b>(x, y) = left.at<cv::Vec3b>(x, y + LUTD2D[depth]);
			}
		}
	}

	return mRight;
}

// Generate the lookup table here to convert depth into disparity
// Don't forget to add the proper parameters!!!
int* Application::LUTDepthToDisparity(float height, float width, int bits, float kNear, float kFar, float tc, float distance)
{
	int* LUTD2D = new int[256];
	// mapping: zp = W ( (m/2^N - 1) ( knear + kfar ) - kfar )
	// disparité (mètres): p = tc * ( 1 - (D / D-zp) )
	// disparité (pixels): ppix = (p * Npix) / W
	for (int depth = 0; depth < 256; depth++) {

		float zp = width*((depth / (pow(2, bits)) * (kNear + kFar) - kFar));
		float p = tc * (1 - (distance / (distance - zp)));
		float ppix = (p*1080) / width;
		LUTD2D[depth] = ppix;
	}
	
	return LUTD2D;
}

// Lookup table to convert 16 bits depth into a displayable 8 bits depth
BYTE* Application::LUTDepthToDisplay(int depthMinReliableDistance, int depthMaxReliableDistance)
{
	BYTE* LUTD2D = new BYTE[USHRT_MAX];

	for(int depth = 0; depth < USHRT_MAX; depth++)
	{
		LUTD2D[depth] = static_cast<BYTE>((depth >= depthMinReliableDistance) && (depth <= depthMaxReliableDistance) ? (depth / cMapDepthToByte) : 0);
	}

	return LUTD2D;
}

// Initializes the default Kinect sensor
HRESULT Application::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);	// Open the default kinect devide

	if(FAILED(hr))	return hr;

	// If we found a kinect
	if(m_pKinectSensor)
	{
		if(SUCCEEDED(hr)) hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);	// Open a coordinate mapper on the current kinect
		
		if(SUCCEEDED(hr)) hr = m_pKinectSensor->Open();	// Open the kinect sensor

		// Open the multisourceframereader to read from the color and the depth streams
		if(SUCCEEDED(hr))	hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color | 
																				FrameSourceTypes::FrameSourceTypes_Depth, 
																				&m_pMultiSourceFrameReader);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	return hr;
}

// Init the default CV window
void Application::InitializeDefaultWindow()
{
	cvNamedWindow("GTI780_TP2", 0);
	cvMoveWindow("GTI780_TP2", 0, 0);
	cvSetWindowProperty("GTI780_TP2", CV_WINDOW_FULLSCREEN, 1);

	HWND win_handle = FindWindow(0, L"GTI780_TP2");

	if (win_handle)
	{
		// Resize
		unsigned int flags = (SWP_SHOWWINDOW | SWP_NOSIZE | SWP_NOMOVE | SWP_NOZORDER);
		flags &= ~SWP_NOSIZE;
		SetWindowPos(win_handle, HWND_NOTOPMOST, cWindowPosX, cWindowPosY, cWindowWidth, cWindowHeight, flags);

		SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
		ShowWindow(win_handle, SW_SHOW);
	}
	else
	{
		printf("Failed FindWindow\n");
	}
}
