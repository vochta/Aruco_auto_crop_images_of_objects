#define WIN32_LEAN_AND_MEAN
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ws2tcpip.h>
#include <stdio.h>
#include <time.h>       /* time_t, struct tm, time, localtime */



using namespace std;
using namespace cv;

#define DEFAULT_BUFLEN 1024
#define DEFAULT_PORT "8000"

string object_name = "stop_sign";


string DEFAULT_OUTPUT_FILE_NAME_BASE = object_name + "_" + "croped_image";

string DEFAULT_OUTPUT_PATH_FOR_ALL_OBJECTS = "D:/Robot/GitHub/Aruco_auto_crop_images_of_objects/Auto_saved_croped_images/";



namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }";
}


void my_replace(std::string& subject, const std::string& search,
                          const std::string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
}

string creat_folder_with_timestamp (string object_name) {
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	//std:: string time_stamp = asctime(timeinfo);
		
	strftime(buffer,80,"%d-%m-%Y %H:%M:%S",timeinfo);
	std::string time_stamp(buffer);

	my_replace(time_stamp, ":", "-");
	my_replace(time_stamp, " ", "_");

	string string_dir_name = DEFAULT_OUTPUT_PATH_FOR_ALL_OBJECTS + "/" + (string)time_stamp;

	LPCSTR dir_name = string_dir_name.c_str();
	// printf (dir_name);


	if (!CreateDirectoryA (dir_name, NULL)) printf ("Error creating folder");;

	string path_to_folder = DEFAULT_OUTPUT_PATH_FOR_ALL_OBJECTS + (string)time_stamp + "/";
	return path_to_folder;
}






/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}



/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

	WSADATA wsaData;
    int iResult;
  
    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

	char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;
	bool first = false; 
	std::vector <char> stream_bytes;
	Mat image;
	char prev_buf = ' ';

	Mat croppedImage;

  
    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }
    }

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } 
	else {
//        inputVideo.open(camId);
          waitTime = 1;

  //*********************************************************************





    
   // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,1), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = 0;//IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // No longer need server socket
    closesocket(ListenSocket);	


    }

    double totalTime = 0;
    int totalIterations = 0;

	iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
	int frame_count = 0;   // total frames recieved
	int saved_croped_images = 0;  // total images auto croped and saved
	string path_to_folder = creat_folder_with_timestamp("test");
    while(inputVideo.grab()||(iResult>0)) {
	
//*********************
    //    printf("Bytes received: %d\n", iResult);
		for (int k = 0; k<iResult; k++)  
		{
			if (k!=0) prev_buf = recvbuf[k-1];
			if (((prev_buf == (char)0xff) && (recvbuf[k] ==(char)0xd8))||first == true) 
			{
				if (!first)	stream_bytes.push_back(prev_buf);
				first = true;

				do {
					stream_bytes.push_back(recvbuf[k]);
					if (k!=0) prev_buf = recvbuf[k-1];
				
					if ((prev_buf == (char)0xff) && (recvbuf[k] == (char)0xd9))
					{
						image = imdecode(stream_bytes, 1);
						if (image.empty()) printf("NULL");
						else { 
							frame_count+=1;   
							imshow("image1", image);
							waitKey(1); // Wait for a keystroke in the window

							Mat imageCopy;
					  //      inputVideo.retrieve(image);

							double tick = (double)getTickCount();

							vector< int > ids;
							vector< vector< Point2f > > corners, rejected;
							vector< Vec3d > rvecs, tvecs;

							// detect markers and estimate pose
							aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
							if(estimatePose && ids.size() > 0)
								aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
																 tvecs);

							double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
							totalTime += currentTime;
							totalIterations++;
							if(totalIterations % 30 == 0) {
								cout << "Detection Time = " << currentTime * 1000 << " ms "
									 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
							}

							// draw results
							image.copyTo(imageCopy);
							if(ids.size() > 0) {
								aruco::drawDetectedMarkers(imageCopy, corners, ids);

								if(estimatePose) {
								 //   for(unsigned int i = 0; i < ids.size(); i++)
								 //       aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
								}
							}
        
							if (corners.size() == 2)     // the number of found markers in the corners array
							{
								cout << corners.size()<< endl;
								cout << corners[0][0].x << endl;
            
								vector< Point2f > marker1 = corners[0];   // points corners 1, 2, 3, 4 of marker #1 (M1)
								vector< Point2f > marker2 = corners[1];   // points corners 1, 2, 3, 4 of marker #2 (M2)
                
								vector <Point2f> object_rectangle;     // points O1, O2, O3, O4
								Rect rectangle_to_crop;   // points C1, C2, C3, C4
            
								object_rectangle.push_back (marker1[0]);     // point O1 = corner №1 of marker №1

                        
								// BEGIN calculation of the coefficients of straight lines L12 and L14 equations:
								float a12_m1 = (marker1[1].y - marker1[0].y)/(marker1[1].x - marker1[0].x);
								float a14_m1 = (marker1[3].y - marker1[0].y)/(marker1[3].x - marker1[0].x);
								float a12_m2 = (marker2[1].y - marker2[0].y)/(marker2[1].x - marker2[0].x);
								float a14_m2 = (marker2[3].y - marker2[0].y)/(marker2[3].x - marker2[0].x);
            
								float b12_m1 = marker1[0].y - marker1[0].x*a12_m1;            
								float b14_m1 = marker1[0].y - marker1[0].x*a14_m1;
								float b12_m2 = marker2[0].y - marker2[0].x*a12_m2;            
								float b14_m2 = marker2[0].y - marker2[0].x*a14_m2;            

								// END calculation of the coefficients of straight lines L12 and L14 equations:
            

								Point2f temp_point;   // point var for calculations
								temp_point.x = (b14_m2 - b12_m1)/(a12_m1-a14_m2);  
								temp_point.y = a12_m1*temp_point.x + b12_m1;

								object_rectangle.push_back (temp_point);  // this is point O2

								object_rectangle.push_back(marker2[0]);     // point O3 = corner №1 of marker №2

								temp_point.x = (b14_m1 - b12_m2)/(a12_m2-a14_m1);
								temp_point.y = a12_m2*temp_point.x + b12_m2;            
            
								object_rectangle.push_back (temp_point);  // this is point O4

								rectangle_to_crop = boundingRect (object_rectangle);

					/*            rectangle_to_crop[0].x = marker1[0].x;
								rectangle_to_crop[0].y = object_rectangle[1].y;
								rectangle_to_crop[1].x = marker2[0].x;
								rectangle_to_crop[1].y = object_rectangle[1].y;
								rectangle_to_crop[2].x = marker2[0].x;
								rectangle_to_crop[2].y = object_rectangle[3].y;
								rectangle_to_crop[3].x = marker1[0].x;
								rectangle_to_crop[3].y = object_rectangle[3].y;
					*/
						/*        rectangle_to_crop[0].x = std::min (object_rectangle[0].x, object_rectangle[1].x); //, std::min(object_rectangle[2].x, object_rectangle[3].x));
						/*        rectangle_to_crop[0].y = min_element (object_rectangle[0].y, object_rectangle[3].y);
								rectangle_to_crop[1].x = max_element (object_rectangle[0].x, object_rectangle[1].x, object_rectangle[2].x, object_rectangle[3].x);
								rectangle_to_crop[1].y = min_element (object_rectangle[0].y, object_rectangle[3].y);
								rectangle_to_crop[2].x = max_element (object_rectangle[0].x, object_rectangle[1].x, object_rectangle[2].x, object_rectangle[3].x);
								rectangle_to_crop[2].y = max_element (object_rectangle[0].y, object_rectangle[3].y);
								rectangle_to_crop[3].x = min_element (object_rectangle[0].x, object_rectangle[1].x, object_rectangle[2].x, object_rectangle[3].x);
								rectangle_to_crop[3].y = max_element (object_rectangle[0].y, object_rectangle[3].y);
						  */      

								cout << rectangle_to_crop << endl;
								
								croppedImage = image(rectangle_to_crop);

								saved_croped_images +=1;
								imshow("out_croped", croppedImage);
								
								// Formulate the filename
								std::ostringstream file_name;
								file_name << path_to_folder << DEFAULT_OUTPUT_FILE_NAME_BASE << saved_croped_images << ".jpg";
								
								imwrite(file_name.str(), croppedImage);
								
							
							}

							if(showRejected && rejected.size() > 0)
								aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

							imshow("out", imageCopy);
							
        
							char key = (char)waitKey(waitTime);
							if(key == 27) break;



						
						}
						stream_bytes.clear();
						first = false;
						k--;
							
					}
					k++;
				} while ((k<iResult) && (first==true));
			}
		}
	iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);


//****************************************************************************



    }

	destroyAllWindows();
    // shutdown the connection since we're done
    iResult = shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();

    return 0;
}
