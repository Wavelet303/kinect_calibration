#include <Windows.h>
#include <Ole2.h>
/*
#include <gl/GL.h>
#include <gl/GLU.h>
#include <gl/glut.h>
*/

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

const int width =  640;
const int height = 480;

using namespace cv;

string work_dir;

// Kinect variables
HANDLE rgbStream;              // The identifier of the Kinect's RGB Camera
INuiSensor* sensor;            // The kinect sensor

bool initKinect(NUI_IMAGE_TYPE m_imageType = NUI_IMAGE_TYPE_COLOR,
	NUI_IMAGE_RESOLUTION m_imageResolution = NUI_IMAGE_RESOLUTION_640x480 ) {
    // Get a working kinect sensor
    int numSensors;
    if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
    if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

    // Initialize sensor
    sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
    sensor->NuiImageStreamOpen(
        m_imageType,             // Depth camera or rgb camera?
        m_imageResolution,    // Image resolution
        0,        // Image `	qqqqdestream flags, e.g. near mode
        2,        // Number of frames to buffer
        NULL,   // Event handle
        &rgbStream);
    return sensor;
}


#define COLOR_INDEX_BLUE            0
#define COLOR_INDEX_GREEN           1
#define COLOR_INDEX_RED             2
#define COLOR_INDEX_ALPHA           3
void SetColor(UINT* pColor, BYTE red, BYTE green, BYTE blue, BYTE alpha = 255){
    if (!pColor)        return;

    BYTE* c = (BYTE*)pColor;
    c[COLOR_INDEX_RED]   = red;
    c[COLOR_INDEX_GREEN] = green;
    c[COLOR_INDEX_BLUE]  = blue;
    c[COLOR_INDEX_ALPHA] = alpha;
}
#define BYTES_PER_PIXEL_INFRARED    2
void CopyInfrared(unsigned char* dest,const BYTE* pImage, UINT size,int width ,int height){
	// Check source buffer size
    if (size != width * height * BYTES_PER_PIXEL_INFRARED)
    {
        return;
    }

    // Allocate buffer for image
    UINT*   pBuffer   = (UINT*)dest; //ResetBuffer(width * height * BYTES_PER_PIXEL_RGB);

    // Initialize pixel pointers
    USHORT* pPixelRun = (USHORT*)pImage;
    USHORT* pPixelEnd = pPixelRun + size / BYTES_PER_PIXEL_INFRARED;

    // Run through pixels
    while (pPixelRun < pPixelEnd) {
        // Convert pixel from 16-bit to 8-bit intensity
        BYTE intensity = (*pPixelRun) >> 8;

        // Set pixel color with R, G and B components all equal to intensity
        SetColor(pBuffer, intensity, intensity, intensity);

        // Move to next pixel
        ++pPixelRun;
        ++pBuffer;
    }
}

bool GetColorData(unsigned char* dest,const BYTE* m_pColorBuffer,const int m_colorBufferPitch,
	UINT size,int width ,int height) 
{
    // Check if image is valid
    if (m_colorBufferPitch == 0||size!=width*height*4)
    {
		printf("m_colorBufferPitch == %d||size!=width*height*4\n",m_colorBufferPitch);
        return false;
    }
    
	for (UINT y = 0; y < height; ++y)   {
		const BYTE* p = &(m_pColorBuffer[y * m_colorBufferPitch]);
		for (UINT x = 0; x < width; ++x){
			*dest++ = m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 0];
			*dest++ = m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 1];
			*dest++ = m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 2];
			*dest++ = m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 3];
		}
	}

    return true;
}

bool getKinectData(unsigned char* dest,NUI_IMAGE_TYPE m_imageType = NUI_IMAGE_TYPE_COLOR
	,const int width = 640,const int height = 480 )
{
    NUI_IMAGE_FRAME imageFrame;
    NUI_LOCKED_RECT LockedRect;
    if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return false;
	bool has_data = false;
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0){		
		switch (m_imageType) {
		case NUI_IMAGE_TYPE_COLOR_RAW_BAYER: 
		case NUI_IMAGE_TYPE_COLOR_INFRARED:
			CopyInfrared(dest,LockedRect.pBits, LockedRect.size,width,height);
			break;
		default:  
			memcpy_s(dest, LockedRect.size, (const BYTE*) (LockedRect.pBits), width*height*4);
	/*		const BYTE* curr = (const BYTE*) LockedRect.pBits;
			const BYTE* dataEnd = curr + (width*height)*4;

			while (curr < dataEnd) {
				*dest++ = *curr++;
			}
			*/
	//		GetColorData(dest,(const BYTE*) (LockedRect.pBits),LockedRect.size,width,height);
		}
		has_data = true;
    }
    texture->UnlockRect(0);
    sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
	return has_data ;
}

//------------------convet to opencv Mat start --------
//http://stackoverflow.com/questions/13003090/kinect-sdk-depth-data-c-to-opencv 
/*
// get a CV_8U matrix from a Kinect depth frame 
cv::Mat * GetDepthImage(USHORT * depthData, int width, int height) 
{
    const int imageSize = width * height; 
    cv::Mat * out = new cv::Mat(height, width, CV_8U) ;
    // map the values to the depth range
    for (int i = 0; i < imageSize; i++)
    {
        // get the lower 8 bits
        USHORT depth =  depthData[i];   
        if (depth >= kLower && depth <= kUpper) 
        {
            float y = c * (depth - kLower); 
            out->at<byte>(i) = (byte) y; 
        }
        else
        {
            out->at<byte>(i) = 0; 
        }
    }
    return out; 
};

// get a CV_8UC4 (RGB) Matrix from Kinect RGB frame
cv::Mat * GetColorImage(unsigned char * bytes, int width, int height)
{
    const unsigned int img_size = width * height * 4; 
    cv::Mat * out = new cv::Mat(height, width, CV_8UC4);

    // copy data
    memcpy(out->data, bytes, img_size); 

    return out; 
}
*/

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

bool detectPattern(const Mat &image,const Pattern pattern,const Size boardSize
	,std::vector<Point2f> &pointbuf,int calibration_flags=0)
{ 
	Mat  viewGray;	
	if(image.channels()>=3)
		cvtColor(image, viewGray, COLOR_BGR2GRAY);
	else viewGray = image;

	calibration_flags = 0;
	calibration_flags = calibration_flags
		| CV_CALIB_CB_NORMALIZE_IMAGE
		| CV_CALIB_CB_FILTER_QUADS
		| CV_CALIB_CB_ADAPTIVE_THRESH;
	//	| CV_CALIB_CB_FAST_CHECK 


	bool found=false;
	switch( pattern )
	{
	case CHESSBOARD:
		found = findChessboardCorners( image, boardSize, pointbuf,calibration_flags	);
		break;
	case CIRCLES_GRID:
		found = findCirclesGrid( image, boardSize, pointbuf );
		break;
	case ASYMMETRIC_CIRCLES_GRID:
		found = findCirclesGrid( image, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID );
		break;
	default:
		return fprintf( stderr, "Unknown pattern type\n" ), -1;
	}

	// improve the found corners' coordinate accuracy
	if( pattern == CHESSBOARD && found) cornerSubPix( viewGray, pointbuf, Size(11,11),
		Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	return found;
}

void test_calibration(){
	namedWindow("ColorImage", WINDOW_AUTOSIZE);

	string filename("ir_000054.png"); //left01.jpg");//
	Pattern pattern = CHESSBOARD;
	Size boardSize; boardSize.width = 11; boardSize.height = 7;
	cv::Mat image= imread(filename);
	std::vector<Point2f> pointbuf;
	bool found  = detectPattern(image,pattern,boardSize,pointbuf);
	if(found){
		//	image2 = image.clone();
            drawChessboardCorners( image, boardSize, Mat(pointbuf), found );			
		}
	imshow("ColorImage", image);
	int key = 0xff & waitKey(0);
}

void printType(Mat &mat) {
         if(mat.depth() == CV_8U)  printf("unsigned char(%d)", mat.channels());
    else if(mat.depth() == CV_8S)  printf("signed char(%d)", mat.channels());
    else if(mat.depth() == CV_16U) printf("unsigned short(%d)", mat.channels());
    else if(mat.depth() == CV_16S) printf("signed short(%d)", mat.channels());
    else if(mat.depth() == CV_32S) printf("signed int(%d)", mat.channels());
    else if(mat.depth() == CV_32F) printf("float(%d)", mat.channels());
    else if(mat.depth() == CV_64F) printf("double(%d)", mat.channels());
    else                           printf("unknown(%d)", mat.channels());
}
void printInfo(Mat &mat) {
    printf("dim(%d, %d)", mat.rows, mat.cols);
    printType(mat);
    printf("\n");
}

inline char *copyPath(char *buf,const string work_dir){	
	int len;
	if(work_dir.size()>0){
		sprintf (buf,work_dir.c_str());	
		len = work_dir.size();
	}
	else{
		sprintf (buf,"./");	
		len = 2;
	}
	return buf+len;
}

int  capture(Size boardSize, Size imageSize,NUI_IMAGE_TYPE m_imageType = NUI_IMAGE_TYPE_COLOR,
	Pattern pattern = CHESSBOARD,int delay = 500,bool save = true)
{
	if (!initKinect(m_imageType)){
		printf("can't open kinect sensor\n");
		return 1;
	}
	
	const unsigned int img_size = imageSize.width * imageSize.height * 4;
	unsigned char *data = new unsigned char[img_size];

	if(imageSize.width<=0||imageSize.height<=0)
		printf("mageSize.width<=0||imageSize.height<=0\n");
	printf("img_size:=%d\n",img_size);

	namedWindow("ColorImage", WINDOW_AUTOSIZE);
	cv::Mat image(imageSize, CV_8UC4);// height, width, CV_8UC4);
	std::vector<cv::Mat> images;
	int i = 0;
	while(1){
		if(!getKinectData(data,m_imageType)){
			int key = 0xff & waitKey(delay);
			if( (key & 255) == 27 )
				break;
			else continue;
		}

		memcpy(image.data, data, img_size); // copy data	

		std::vector<Point2f> pointbuf;
		bool found  = detectPattern(image,pattern,boardSize,pointbuf);
	
		Mat  image2;	
		if(found){
			image2 = image.clone();
            drawChessboardCorners( image, boardSize, Mat(pointbuf), found );			
		}
		imshow("ColorImage", image);

		int key = 0xff & waitKey(delay);
        if( (key & 255) == 27 )
            break;	
		
		if(key=='s'&&found){
			char buf[4096];			
			char *p = copyPath(buf,work_dir);

			if(m_imageType != NUI_IMAGE_TYPE_COLOR_INFRARED)
				sprintf (p, "%06d.bmp", (int)i); // "./%06d.bmp", (int)i);
			else 
				sprintf (p, "ir_%06d.bmp", (int)i); //"./ir_%06d.bmp", (int)i);	
			cv::imwrite (buf, image2);
			printf ("writing: %s\n", buf);
			//	images.push_back(image2); 
		//	printInfo(image2);
		//	cv::imshow("color", image2);
		//	cv::waitKey(100);
			i++;
		}
		else if(key=='u'){
			if(images.size()>0)
				images.erase( images.end()-1);
		} 
	}
	/*
	if(save){
		printf("saving %d images to harddisk\n",images.size());
		for (i = 0 ; i<images.size();i++){
			char buf[4096];
			if(m_imageType != NUI_IMAGE_TYPE_COLOR_INFRARED)
				sprintf (buf, "./%06d.bmp", (int)i);
			else 
				sprintf (buf, "./ir_%06d.bmp", (int)i);
			cv::imwrite (buf, images[i]);
			printf ("writing: %s\n", buf);
		}
	}
	*/
	delete[] data;
}

const char * usage =
" \nexample command line for capture from a kinect.\n"
"   kinect_capture  -w 4 -h 5 -ir 0 -s 3.02 -o camera.yml \n"
;
static void help()
{
	printf( "This is a kinect calibration.\n"
		"Usage: calibration\n"
		"     -w <board_width>         # the number of inner corners per one of board dimension\n"
		"     -h <board_height>        # the number of inner corners per another board dimension\n"
		"     [-ir <ir mode>]          # the type of colr image: NUI_IMAGE_TYPE_COLOR or NUI_IMAGE_TYPE_COLOR_INFRARED\n"
		"     [-pt <pattern>]          # the type of pattern: chessboard or circles' grid\n"
		"     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
        "                              # (used only for video capturing)\n" 
		"     [-s <squareSize>]       # square size in some user-defined units (1 by default)\n"
		"     [-zt]                    # assume zero tangential distortion\n"
        "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
        "     [-p]                     # fix the principal point at the center\n"
		"     [-v]                     # flip the captured images around the horizontal axis\n"
	    "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"	 
		"\n" );
	printf("\n%s",usage);	
}


static bool runAndSave(const string& outputFilename,                
                Size boardSize, float squareSize,Pattern patternType, 
                Mat& cameraMatrix,Mat& distCoeffs,NUI_IMAGE_TYPE m_imageType = NUI_IMAGE_TYPE_COLOR,
				const string& inputFilename = "",
				float aspectRatio = 1.f, int flags=0,
				bool writeExtrinsics = false, bool writePoints = false );
double  extrinsicsCalibration(const string& imagePairFilename,
	const string& extrinsicFilename,const cv::Size boardSize,const double squareSize );


int main(int argc, char** argv)
{	
//	int main_alignment_ms(int argc, char** argv);	return main_alignment_ms(argc,argv);
//	int test_png_lossless(); return test_png_lossless();
	
//	test_calibration();return 0;
	Pattern pattern = CHESSBOARD;
	Size boardSize, imageSize;	
	
	int delay = 1000;
	int IR = 0,resolution = 2;
	int ExtrinsicFlag = 0;
	NUI_IMAGE_TYPE m_imageType = NUI_IMAGE_TYPE_COLOR;	
	boardSize.width = 11; boardSize.height = 7;
	imageSize.width = 640; imageSize.height = 480;

	string outputFilename = "camera.ymal";
	string imagePairFilename = "image_pairs.xml";	

	bool flipVertical = false;
	float aspectRatio = 1.f;
	int flags=0;
	float squareSize = 0.f;

	if(argc<3){
		help();
		return -1;
	}

	int i;
	for( i = 1; i < argc; i++ )
    {
        const char* s = argv[i];
        if( strcmp( s, "-w" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
                return fprintf( stderr, "Invalid board width\n" ), -1;
        }
        else if( strcmp( s, "-h" ) == 0 )
        {
			if( sscanf( argv[++i], "%u", &boardSize.height) != 1 || boardSize.height <= 0 )
                return fprintf( stderr, "Invalid board height\n" ), -1;          
        }
		else if( strcmp( s, "-s" ) == 0 )
        {
            if( sscanf( argv[++i], "%f", &squareSize ) != 1 || squareSize <= 0 )
                return fprintf( stderr, "Invalid board square width\n" ), -1;
        }
		else if( strcmp( s, "-ex" ) == 0 )
        {
             if( sscanf( argv[++i], "%d", &ExtrinsicFlag ) != 1  )
                return fprintf( stderr, "Invalid ExtrinsicFlag \n" ), -1;			 
        }	
		else if( strcmp( s, "-ir" ) == 0 )
        {
             if( sscanf( argv[++i], "%d", &IR ) != 1  )
                return fprintf( stderr, "Invalid IR\n" ), -1;
			 if(IR){
				 m_imageType = NUI_IMAGE_TYPE_COLOR_INFRARED;
				 printf("m_imageType=:%d",m_imageType);
			 }
        }					
		else if( strcmp( s, "-d" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &delay ) != 1 || delay <= 0 )
                return printf("Invalid delay\n" ), -1;
        }
		else if( strcmp( s, "-a" ) == 0 )
        {
            if( sscanf( argv[++i], "%f", &aspectRatio ) != 1 || aspectRatio <= 0 )
                return printf("Invalid aspect ratio\n" ), -1;
            flags |= CV_CALIB_FIX_ASPECT_RATIO;
        }
		 else if( strcmp( s, "-zt" ) == 0 )
        {
            flags |= CV_CALIB_ZERO_TANGENT_DIST;
        }
        else if( strcmp( s, "-p" ) == 0 )
        {
            flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
        }
		  else if( strcmp( s, "-zt" ) == 0 )
        {
            flags |= CV_CALIB_ZERO_TANGENT_DIST;
        }
         else if( strcmp( s, "-v" ) == 0 )
        {
            flipVertical = true;
        }
		 else if( strcmp( s, "-o" ) == 0 )
        {
            outputFilename = argv[++i];
        }
		else if( strcmp( s, "-ip" ) == 0 )
        {
            imagePairFilename = argv[++i];
        }
		else if( strcmp( s, "-dir" ) == 0 )
        {
            work_dir = argv[++i];
			if(work_dir.back()!='\/'&&work_dir.back()!='\\')
				work_dir+="\/";
        }

	}
	
	printf("board_width = :%d\t board_height = :%d\n",boardSize.width,boardSize.height);
	printf("image_width = :%d\t image_height = :%d\n",imageSize.width,imageSize.height);
	printf("squareSize = :%f\n",squareSize);
	if(squareSize==0){
		capture(boardSize, imageSize,m_imageType, pattern , delay );		
	}
	else if(ExtrinsicFlag){
		printf("start to extrinsic calibration...\n");
		extrinsicsCalibration(imagePairFilename,outputFilename,	boardSize,squareSize);
	}
	else{
		Mat cameraMatrix,distCoeffs;
		runAndSave(outputFilename,boardSize,squareSize,pattern,cameraMatrix,distCoeffs,m_imageType,
			"",	aspectRatio,flags);
	}	
	cv::waitKey(0);
	return 0;
}


//-----------------calibration----------
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}
static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch(patternType)
    {
      case CHESSBOARD:
      case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

      case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

      default:
        CV_Error(CV_StsBadArg, "Unknown pattern type\n");
    }
}
static bool intrinsicCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize, Pattern patternType,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}


static void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

#include <fstream>
static bool readStringList( const string& filename, vector<string>& l )
{
	l.resize(0);
	string ext = filename.substr(filename.size()-3,filename.size());
	if(ext=="txt"){
		std::ifstream iF(filename);
		if(!iF) return false;
		string file;
		while(iF>>file)
			l.push_back(file);
		return true;
	}
	else{		
		FileStorage fs(filename, FileStorage::READ);
		if( !fs.isOpened() )
			return false;
		FileNode n = fs.getFirstTopLevelNode();
		if( n.type() != FileNode::SEQ )
			return false;
		FileNodeIterator it = n.begin(), it_end = n.end();
		for( ; it != it_end; ++it )
			l.push_back((string)*it);
		return true;
	}
  
}

static bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, float squareSize,Pattern patternType, 
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = intrinsicCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if( ok )
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
    return ok;
}

static bool runAndSave(const string& outputFilename,                
                Size boardSize, float squareSize,Pattern pattern, 
                Mat& cameraMatrix,Mat& distCoeffs, NUI_IMAGE_TYPE m_imageType,
				const string& inputFilename ,
				float aspectRatio , int flags,
				bool writeExtrinsics , bool writePoints  )
{
	vector<vector<Point2f> > imagePoints;
	vector<string> imageList;
	if(inputFilename.size()>0){		
		readStringList(inputFilename, imageList);
		printf("has read %d images!\n",imageList.size());
	}
	Size imageSize;

	bool flipVertical = false;

	namedWindow("ColorImage", WINDOW_AUTOSIZE);
	if(imageList.size()==0){
		for (int i = 0 ; i<100;i++){
			cv::Mat view,viewGray;
			char buf[4096];
			char *p = copyPath(buf,work_dir);
			if(m_imageType != NUI_IMAGE_TYPE_COLOR_INFRARED)
				sprintf (p, "%06d.bmp", (int)i); // "./%06d.bmp", (int)i);
			else 
				sprintf (p, "ir_%06d.bmp", (int)i); //"./ir_%06d.bmp", (int)i);
			printf("read image %s...\n",buf);
			view = cv::imread (buf);

			if(!view.data) continue;	

			imageSize = view.size();

			if( flipVertical )
				flip( view, view, 0 );

			std::vector<Point2f> pointbuf;
		    bool found  = detectPattern(view,pattern,boardSize,pointbuf);

			if( found ){
				imagePoints.push_back(pointbuf);
				drawChessboardCorners( view, boardSize, Mat(pointbuf), found );
			}

			imshow("ColorImage", view);
			waitKey(500);
		}
	}

	if(imagePoints.size()<3){
		printf("%d images are not enough!\n ",imagePoints.size());
	}
	
	return runAndSave(outputFilename,imagePoints,imageSize,boardSize,
		squareSize,pattern,aspectRatio,flags,cameraMatrix,distCoeffs,
		writeExtrinsics,  writePoints);
}

//----------------
#define OUT_ERROR printf
#define OUT_INFO cout


double extrinsicsCalibration(const std::vector<std::vector<cv::Point3f> > pointsBoard,
	const std::vector<std::vector<cv::Point2f> > &pointsIr,
	const std::vector<std::vector<cv::Point2f> > &pointsColor,
	cv::Mat& cameraMatrixIr,cv::Mat& distortionIr,
	cv::Mat& cameraMatrixColor, cv::Mat& distortionColor,	
	cv::Size imageSize,cv::Mat& rotation,cv::Mat& translation,
	cv::Mat& essential,cv::Mat& fundamental,
	TermCriteria termCriteria =  TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON))
{
	if(pointsColor.size() != pointsIr.size())
    {
      OUT_ERROR("number of detected color and ir patterns does not match!");
      return -1;
    }
    if(pointsColor.empty() || pointsIr.empty())
    {
      OUT_ERROR("no data for calibration provided!");
      return -2;
    }
    double error;

	std::cout<<"\nCamera Matrix Color:" << std::endl << cameraMatrixColor;
    std::cout<<"\nDistortion Coeeficients Color:" << std::endl << distortionColor << std::endl;
    std::cout<<"\nCamera Matrix Ir:" << std::endl << cameraMatrixIr;
    std::cout<<"\nDistortion Coeeficients Ir:" << std::endl << distortionIr << std::endl;

    std::cout<<"calibrating Color and Ir extrinsics...\n";
#if CV_MAJOR_VERSION == 2
    error = cv::stereoCalibrate(pointsBoard, pointsIr, pointsColor, 
		cameraMatrixIr, distortionIr, cameraMatrixColor, distortionColor, imageSize,
                                rotation, translation, essential, fundamental, termCriteria, CV_CALIB_USE_INTRINSIC_GUESS);//cv::CALIB_FIX_INTRINSIC+CV_CALIB_USE_INTRINSIC_GUESS);
#elif CV_MAJOR_VERSION == 3
    error = cv::stereoCalibrate(pointsBoard, pointsIr, pointsColor, cameraMatrixIr, distortionIr, cameraMatrixColor, distortionColor, sizeColor,
                                rotation, translation, essential, fundamental, cv::CALIB_FIX_INTRINSIC, termCriteria);
#endif
    std::cout<<"re-projection error: " << error << std::endl;

    std::cout<<"\nRotation:" << std::endl << rotation;
    std::cout<<"\nTranslation:" << std::endl << translation;
    std::cout<<"\nEssential:" << std::endl << essential;
    std::cout<<"\nFundamental:" << std::endl << fundamental << std::endl;

	return error;
}

bool readIntrinsicParames(const string& cameraFilename,
	cv::Mat& cameraMatrix,cv::Mat& distCoeffs)
{
	cv::FileStorage fs;
    if(fs.open(cameraFilename, cv::FileStorage::READ))
    {
      fs["camera_matrix"] >> cameraMatrix;
      fs["distortion_coefficients"] >> distCoeffs;    
      fs.release();
	  return true;
    }
    else
    {
      OUT_ERROR("couldn't load color calibration data!");
      return false;
    }
}

static void saveExtrinsicParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, 
					   cv::Mat& cameraMatrix0,cv::Mat& distortion0,
					   cv::Mat& cameraMatrix1, cv::Mat& distortion1,
					   cv::Mat rotation,cv::Mat translation,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    
	fs << "cameraMatrix0" << cameraMatrix0;
    fs << "distortion0" << distortion0;

	fs << "cameraMatrix1" << cameraMatrix1;
	fs << "distortion1" << distortion1;   

    fs << "rotation" << rotation;
    fs << "translation" << translation;

    fs << "avg_reprojection_error" << totalAvgErr;
 }

double  extrinsicsCalibration(const string& imagePairFilename,
	const string& extrinsicFilename,const cv::Size boardSize,const double squareSize )
{
	vector<string> imagelist2;
	if(!readStringList(imagePairFilename, imagelist2)){
		printf("can't read imagePairFilename:%s\n",imagePairFilename.c_str());
		return -1.;
	}
	else if(imagelist2.size()<4){
		printf(" %s no enough images\n",imagePairFilename.c_str());
		return -2.;
	}
	vector<string> cameraFiles(imagelist2.begin(),imagelist2.begin()+2);
	vector<string> imagelist(imagelist2.begin()+2,imagelist2.end());

	cv::Mat cameraMatrix[2],distortion[2];
	for(int i = 0 ; i<2;i++){
		if(!readIntrinsicParames(cameraFiles[i],cameraMatrix[i],distortion[i])){
			printf("can't read camera file :%s\n",cameraFiles[i].c_str());
			return -3;
		}
	}	


	double error;
	bool displayCorners = true;
    const int maxScale = 2;

	cv::Mat rotation, translation, essential, fundamental, disparity;
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;
	int nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;
	int i,j,k;
	
    for( i = j = 0; i < nimages; i++ )
    {
		for( k = 0; k < 2; k++ )
		{
			const string& filename = imagelist[i*2+k];
			Mat img = imread(filename, 0);
			if(img.empty())
				break;
			if( imageSize == Size() )
				imageSize = img.size();
			else if( img.size() != imageSize )
			{
				std::cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for( int scale = 1; scale <= maxScale; scale++ )
			{
				Mat timg;
				if( scale == 1 )
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
				if( found )
				{
					if( scale > 1 )
					{
						Mat cornersMat(corners);
						cornersMat *= 1./scale;
					}
					break;
				}
			}
			if( displayCorners )
			{
				std::cout << filename << std::endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640./MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if( !found )
				break;
			cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
				TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
				30, 0.01));
		}
		if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
			j++;
		}
	}
	std::cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        std::cout << "Error: too little pairs to run the calibration\n";
        return -4;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

	for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    std::cout << "Running stereo calibration ...\n";
	error = extrinsicsCalibration(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distortion[0],cameraMatrix[1], distortion[1],
		imageSize,rotation, translation, essential, fundamental);

	saveExtrinsicParams(extrinsicFilename,imageSize,boardSize,squareSize,
		cameraMatrix[0], distortion[0],cameraMatrix[1], distortion[1],
		rotation, translation,error);
	return error;
}


bool readExtrinsicParams( const string& filename,
	                   cv::Mat& cameraMatrix0,cv::Mat& distortion0,
					   cv::Mat& cameraMatrix1, cv::Mat& distortion1,
					   cv::Mat& rotation,cv::Mat& translation)
{
	cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
      fs["cameraMatrix0"] >> cameraMatrix0;
	  fs["distortion0"] >> distortion0;
      fs["cameraMatrix1"] >> cameraMatrix1;
	  fs["distortion1"] >> distortion1;

	  fs["rotation"] >> rotation;
	  fs["translation"] >> translation;

      fs.release();
	  return true;
    }
    else
    {
      OUT_ERROR("couldn't load color calibration data!\n");
      return false;
    }
}