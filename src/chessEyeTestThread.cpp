#include "chessEyeTestThread.h"
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <unistd.h>
#include <iomanip>

#define DIST_FROM_THRES 0.0

string int_to_string( const int a )
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

jointsVelocities::jointsVelocities()
{
    name = "";
    jnts.resize(6,0.0);
    vels.resize(6,0.0);
}

jointsVelocities::jointsVelocities(string _name, Vector _jnts, Vector _vels)
{
    name = _name;
    jnts = _jnts;
    vels = _vels;
}


jointsVelocities & jointsVelocities::operator= (const jointsVelocities &jv)
{
    name = jv.name;
    jnts = jv.jnts;
    vels = jv.vels;
}

void jointsVelocities::print()
{
    printf("*** %s\n",name.c_str());
    printf("joints:     %s\n",jnts.toString().c_str());
    printf("velocities: %s\n",vels.toString().c_str());
    printf("**********\n");
}

chessEyeTestThread::chessEyeTestThread(int _rate, string _name, string _robot, int _v,
                                       int _nG, double _eps, double _tO, const ResourceFinder &_rf) :
                                       RateThread(_rate), name(_name), robot(_robot),
                                       verbosity(_v), numWaypoints(_nG), epsilon(_eps), timeout(_tO)
{
    imagePortInLeft  = new BufferedPort<ImageOf<PixelRgb> >;
    imagePortInRight = new BufferedPort<ImageOf<PixelRgb> >;
    outPortLeft      = new BufferedPort<ImageOf<PixelRgb> >;
    outPortRight     = new BufferedPort<ImageOf<PixelRgb> >;

    disparityPortIn  = new BufferedPort<ImageOf<PixelBgr> >;
    disparityPortOut = new BufferedPort<ImageOf<PixelBgr> >;

    ResourceFinder &rf=const_cast<ResourceFinder&>(_rf);
    step             =   0;
    currentWaypoint  =   0;

    searchCounter    =  0;
    searchCounterMax = 50;

    mutex			 = new Semaphore(1);

    //******************* TYPE ******************
        if (rf.check("type"))
        {
            type = rf.find("type").asInt();
            printf(("*** "+name+": type set to %g\n").c_str(),type);
        }
        else printf(("*** "+name+": could not find type option in the config file; using %s as default\n").c_str(),type);

        if (type==0)
        {
            testUnderExecution = &chessEyeTestThread::searchForGrid;
            textToPrint        = "SEARCHING FOR GRID\n";
        }
        else if (type==1)
        {
            testUnderExecution = &chessEyeTestThread::compareDisparity;
            textToPrint        = "CHECKING DISPARITY\n";
        }

    //******************* TYPE ******************
        iterations=rf.check("iterations",Value(1)).asInt();
        printf(("*** "+name+": number of iterations set to %g\n").c_str(),iterations);

    //******************* CALIB THINGS ******************
    //******************* BOARDWIDTH ******************
        if (rf.check("boardWidth"))
        {
            boardWidth = rf.find("boardWidth").asInt();
            printf(("*** "+name+": boardWidth set to %i\n").c_str(),boardWidth);
        }
        else printf(("*** "+name+": could not find boardWidth option in the config file; using %i as default\n").c_str(),boardWidth);
    //******************* BOARDHEIGHT ******************
        if (rf.check("boardHeight"))
        {
            boardHeight = rf.find("boardHeight").asInt();
            printf(("*** "+name+": boardHeight set to %i\n").c_str(),boardHeight);
        }
        else printf(("*** "+name+": could not find boardHeight option in the config file; using %i as default\n").c_str(),boardHeight);
    // //******************* BOARDSIZE ******************
    //     if (rf.check("boardSize"))
    //     {
    //         boardSize = rf.find("boardSize").asDouble();
    //         printf(("*** "+name+": boardSize set to %g\n").c_str(),boardSize);
    //     }
    //     else printf(("*** "+name+": could not find boardSize option in the config file; using %s as default\n").c_str(),boardSize);
    //*************** INPUT CAMERAS ***************
        inputL   =rf.check("leftCamera", Value("/camcalib/left/out")).asString().c_str();
        inputR   =rf.check("rightCamera",Value("/camcalib/right/out")).asString().c_str();
        inputDisp=rf.check("dispPort",Value("/stereoDisparity/disparity:o")).asString().c_str();
        inputR="/"+robot+inputR;
        inputL="/"+robot+inputL;
        printMessage(0,"inputL: %s\n",inputL.c_str());
        printMessage(0,"inputR: %s\n",inputR.c_str());
        printMessage(0,"inputDisp: %s\n",inputDisp.c_str());

    //******************* GROUPS ******************
        for (int j = 0; j < iterations; j++)
        {
	        for (int i = 0; i < numWaypoints; i++)
	        {
	            string waypoIntname = "WAYPOINT_"+int_to_string(i);
	            
	            Bottle &b = rf.findGroup(waypoIntname.c_str());
	            if (!b.isNull())
	            {
	                printMessage(2,"%s found: %s\n",waypoIntname.c_str(),b.toString().c_str());
	                Vector _jnts(6,0.0);
	                Vector _vels(6,0.0);
	                Bottle *bj = b.find("jnts").asList();
	                Bottle *bv = b.find("vels").asList();

	                for (int j = 0; j < _jnts.size(); j++)
	                {
	                    _jnts[j] = bj->get(j).asDouble();
	                    _vels[j] = bv->get(j).asDouble();
	                }
	                jvWaypoints.push_back(jointsVelocities(waypoIntname,_jnts,_vels));
	                jvWaypoints[i].print();
	            }
	        }
        }

        numWaypoints = numWaypoints*iterations; // the iterations will be multipliers of the numbwaipoints

        jvWaypoints.push_back(jvWaypoints[0]); // the last group will be the first one
        numWaypoints++;


    //******************* PATH ******************
        path = rf.getHomeContextPath().c_str();
        
        // Let's add some contextual info (the date) to the file created!
        time_t now = time(0);
        tm *ltm = localtime(&now);
        string time = int_to_string(1900 + ltm->tm_year)+"_"+int_to_string(1+ltm->tm_mon)+"_"+
                      int_to_string(ltm->tm_mday)+"_"+int_to_string(1+ltm->tm_hour)+"_"+
                      int_to_string(1+ltm->tm_min);

        path = path+"/"+time+"/";

        printMessage(0,"Storing file set to: %s\n", path.c_str());

        configfile = rf.findFile("from");
        printf("%s\n", configfile.c_str());

        // configfile = rf.find("from").asString();
        // printf("%s\n", configfile.c_str());
}

bool chessEyeTestThread::threadInit()
{
    bool ok = 1;
    if (! (imagePortInLeft  -> open(("/"+name+"/imgL:i").c_str()) &&
           imagePortInRight -> open(("/"+name+"/imgR:i").c_str()) &&
           outPortLeft      -> open(("/"+name+"/imgL:o").c_str()) &&
           outPortRight     -> open(("/"+name+"/imgR:o").c_str()) &&
           disparityPortIn  -> open(("/"+name+"/disp:i").c_str()) &&
           disparityPortOut -> open(("/"+name+"/disp:o").c_str())) )
    {   
        printMessage(0,"ERROR! unable to open one of the ports!\n");
        return false;
    }
    else
    {
        ok=ok && Network::connect(inputL.c_str(),("/"+name+"/imgL:i").c_str());
        ok=ok && Network::connect(inputR.c_str(),("/"+name+"/imgR:i").c_str());
        Network::connect(inputDisp.c_str(),("/"+name+"/disp:i").c_str());
        Network::connect(("/"+name+"/imgL:o").c_str(),"/chessEyeTestInL");    // this one is not mandatory
        Network::connect(("/"+name+"/imgR:o").c_str(),"/chessEyeTestInR");    // this one is not mandatory
        Network::connect(("/"+name+"/disp:o").c_str(),"/chessEyeTestDisp");   // this one is not mandatory
    }

    if (!ok)
    {
        printMessage(0,"ERROR! unable to connect the input ports!\n");
        return false;
    }

    Property OptH;
    OptH.put("robot",  robot.c_str());
    OptH.put("part",   "head");
    OptH.put("device", "remote_controlboard");
    OptH.put("remote",("/"+robot+"/head").c_str());
    OptH.put("local", ("/"+name +"/head").c_str());

    if (!ddH.open(OptH))
    {
        printMessage(0,"ERROR: could not open head PolyDriver!\n");
        return false;
    }

    // open the view
    if (ddH.isValid())
    {
        ok = ok && ddH.view(iencsH);
        ok = ok && ddH.view(iposH);
        ok = ok && ddH.view(ilimH);
    }

    if (!ok)
    {
        printMessage(0,"\nERROR: Problems acquiring head interfaces!!!!\n");
        return false;
    }

    iencsH->getAxes(&jntsH);
    encsH = new Vector(jntsH,0.0);
    encsH_STRT.resize(jntsH,0.0);
    encsH_END.resize(jntsH,0.0);

    timeflag = 0;

    alignJointsBounds();

    return true;
}

void chessEyeTestThread::run()
{
	if (step != 6)
	{
		printMessage(2,"currentWaypoint: %i numWaypoints: %i step: %i\n",currentWaypoint,numWaypoints,step);
	}
	// printMessage(0,"I'm here\n");
    if (checkMotionDone())
    {
    	bool res = 0;
        switch (step)
        {
            case 0:
                printMessage(0,"STARTUP\n");
                step++;
                break;
            case 1:
                printMessage(0,"Going to group %i...\n", currentWaypoint);
                if (verbosity>=1)
                    jvWaypoints[currentWaypoint].print();
                goToWaypoint(currentWaypoint);
                step++;
                break;
            case 2:
                printMessage(0,"Group %i accomplished!\n", currentWaypoint);

                if ((currentWaypoint==0 || currentWaypoint==numWaypoints-1))
                    step++;
                else 
                    step = 1;

                currentWaypoint++;
                break;
            case 3:
            	//string s = test==0?"SEARCHING FOR GRID\n":"CHECKING DISPARITY\n";
                printMessage(0,textToPrint.c_str());
                step++;
            	if (currentWaypoint == numWaypoints)
            	{
            		printMessage(0,"Delaying\n");
            		Time::delay(10.0);
            	}
                break;
            case 4:
            	res = (this->*testUnderExecution)();
            	if (res)
            	{
	                if (currentWaypoint == 1)
	                {
	                	step          = 1;
	                	searchCounter = 0;
	                }
	                else if (currentWaypoint == numWaypoints)
	                    step++;
            		printMessage(1,"Success!\n");
            	}
                break;
            case 5:
                if (robot == "icub")
                    printResults();

                printMessage(0,"FINISH\n");
                step++;
                break;
            case 6:
                break;
            default:
                printMessage(0,"ERROR!!! chessEyeTestThread should never be here!!!\nStep: %d",step);
                delay(1);
                break;
        }
    }
}

bool chessEyeTestThread::compareDisparity()
{
    if (robot == "icubSim")
        return true;

    imageDispIn = new ImageOf<PixelBgr>;
    
    ImageOf<PixelBgr> *tmp = disparityPortIn->read(false);

    if(tmp!=NULL)
    {
        *imageDispIn=*tmp;
        IplImage *iplDispIn = (IplImage*) imageDispIn->getIplImage();
        cv::Mat matDispIn(iplDispIn);
        cv::Mat bwDispIn;
        cvtColor(matDispIn, bwDispIn, CV_RGB2GRAY);

        printMessage(3,"Writing images...\n");

        if (currentWaypoint==1)
        {
            bwDispIn_STRT = bwDispIn;

            ImageOf<PixelBgr>& outim=disparityPortOut->prepare();
            outim.wrapIplImage(iplDispIn);
            disparityPortOut->write();
        }
        else if (currentWaypoint==numWaypoints)
        {
            bwDispIn_END = bwDispIn;
            cv::Mat bwDispIn_DIFF  = bwDispIn_END-bwDispIn_STRT;
            cv::Mat rgbDispIn_DIFF; 
			// cout << bwDispIn_DIFF << endl; fflush(stdout);

            cvtColor(bwDispIn_DIFF,rgbDispIn_DIFF,CV_GRAY2RGB);
            IplImage output=rgbDispIn_DIFF;

            ImageOf<PixelBgr>& outim=disparityPortOut->prepare();
            outim.wrapIplImage(&output);
            disparityPortOut->write();
        }

        searchCounter++;
        if (searchCounter == searchCounterMax)
        {
        	return true;
        }
    }

    delete imageDispIn;
    return false;
}

void chessEyeTestThread::printResults()
{
    std::vector<cv::Point2f> resR;
    std::vector<cv::Point2f> resL;

    double avg_x_R = 0.0, avg_y_R = 0.0;
    double avg_x_L = 0.0, avg_y_L = 0.0;

    // double std_x_R = 0.0, std_y_R = 0.0;
    // double std_x_L = 0.0, std_y_L = 0.0;

    double min_x_R = 320.0, min_y_R = 240.0;
    double min_x_L = 320.0, min_y_L = 240.0;

    double max_x_R = 0.0, max_y_R = 0.0;
    double max_x_L = 0.0, max_y_L = 0.0;

    string filename = path+"results.log";
	ofstream myfile;
	myfile.open (filename.c_str(), ios::app);

    printMessage(0,"RESULTS:\n");

    myfile << "RESULTS:\n";

    printMessage(2,"Right image:\n");
    myfile << "Right image:\n";
    // Calculation of the average, the min and the max for the right image
    for (int i = 0; i < pointbufR_END.size(); i++)
    {
        resR.push_back(pointbufR_END[i] - pointbufR_STRT[i]);
        resR[i].x = fabs(resR[i].x);
        resR[i].y = fabs(resR[i].y);
        avg_x_R += resR[i].x;
        avg_y_R += resR[i].y;

        if (resR[i].x < min_x_R)
            min_x_R = resR[i].x;
        if (resR[i].y < min_y_R)
            min_y_R = resR[i].y;

        if (resR[i].x > max_x_R)
            max_x_R = resR[i].x;
        if (resR[i].y > max_y_R)
            max_y_R = resR[i].y;

        printMessage(2,"Corner #%i \t error_x %g \t error_y %g\n",i,resR[i].x,resR[i].y);
        myfile << "Corner #" << i << " \t error_x " << resR[i].x << " \t error_y " << resR[i].y << "\n";
    }
    avg_x_R /= resR.size();
    avg_y_R /= resR.size();
    // Calculation of the standard deviation
    // for (int i = 0; i < pointbufR_END.size(); i++)
    // {
    //     std_x_R += (resR[i].x - avg_x_R)*(resR[i].x - avg_x_R);
    //     std_y_R += (resR[i].y - avg_y_R)*(resR[i].y - avg_y_R);
    // }
    // std_x_R /= (resR.size()-1);

    printMessage(2,"Left  image:\n");
    myfile << "Left image:\n";
    // Calculation of the average, the min and the max for the left image
    for (int i = 0; i < pointbufL_END.size(); i++)
    {
        resL.push_back(pointbufL_END[i] - pointbufL_STRT[i]);
        resL[i].x = fabs(resL[i].x);
        resL[i].y = fabs(resL[i].y);
        avg_x_L += resL[i].x;
        avg_y_L += resL[i].y;

        if (resL[i].x < min_x_L)
            min_x_L = resL[i].x;
        if (resL[i].y < min_y_L)
            min_y_L = resL[i].y;

        if (resL[i].x > max_x_L)
            max_x_L = resL[i].x;
        if (resL[i].y > max_y_L)
            max_y_L = resL[i].y;

        printMessage(2,"Corner #%i \t error_x %g \t error_y %g\n",i,resL[i].x,resL[i].y);
        myfile << "Corner #" << i << " \t error_x " << resL[i].x << " \t error_y " << resL[i].y << "\n";
    }
    avg_x_L /= resL.size();
    avg_y_L /= resL.size();
    // // Calculation of the standard deviation
    // for (int i = 0; i < pointbufL_END.size(); i++)
    // {
    //     std_x_L += (resL[i].x - avg_x_L)*(resL[i].x - avg_x_L);
    //     std_y_L += (resL[i].y - avg_y_L)*(resL[i].y - avg_y_L);
    // }
    // std_x_L /= (resL.size());

    printMessage(0,"Right image:\n");
    printMessage(0,"      min_x: %g \t min_y: %g\n",min_x_R,min_y_R);
    printMessage(0,"      max_x: %g \t max_y: %g\n",max_x_R,max_y_R);
    printMessage(0,"      avg_x: %g \t avg_y: %g\n",avg_x_R,avg_y_R);
    // printMessage(0,"      std_x: %g \t std_y: %g\n",std_x_R,std_y_R);
    printMessage(0,"Left image:\n");
    printMessage(0,"      min_x: %g \t min_y: %g\n",min_x_L,min_y_L);
    printMessage(0,"      max_x: %g \t max_y: %g\n",max_x_L,max_y_L);
    printMessage(0,"      avg_x: %g \t avg_y: %g\n",avg_x_L,avg_y_L);
    // printMessage(0,"      std_x: %g \t std_y: %g\n",std_x_L,std_y_L);
    printMessage(0,"Encoders        START:\n%s\n",(encsH_STRT).toString().c_str());
    printMessage(0,"Encoders          END:\n%s\n",(encsH_END).toString().c_str());
    printMessage(0,"Encoders' differences:\n%s\n",(encsH_END-encsH_STRT).toString().c_str());

    myfile << "Configuration file:" << endl;
    myfile << configfile << endl;
	myfile << "Right image:\n";
	myfile << "      min_x: " << min_x_R << " \t min_y: " << min_y_R << "\n";
	myfile << "      max_x: " << max_x_R << " \t max_y: " << max_y_R << "\n";
	myfile << "      avg_x: " << avg_x_R << " \t avg_y: " << avg_y_R << "\n";
	// myfile << "      std_x: " << std_x_R << " \t std_y: " << std_y_R << "\n";
	myfile << "Left image:\n";
	myfile << "      min_x: " << min_x_L << " \t min_y: " << min_y_L << "\n";
	myfile << "      max_x: " << max_x_L << " \t max_y: " << max_y_L << "\n";
	myfile << "      avg_x: " << avg_x_L << " \t avg_y: " << avg_y_L << "\n";
	// myfile << "      std_x: " << std_x_L << " \t std_y: " << std_y_L << "\n";
	myfile << "Encoders        START:\n" << (encsH_STRT).toString() << "\n";
	myfile << "Encoders          END:\n" << (encsH_END).toString() << "\n";
	myfile << "Encoders' differences:\n" << (encsH_END-encsH_STRT).toString() << "\n";
	myfile.close();
}

bool chessEyeTestThread::checkTS(double TSLeft, double TSRight, double th)
{
    double diff=fabs(TSLeft-TSRight);
    return true;
    if(diff <th)
        return true;
    else return false;
}

bool chessEyeTestThread::searchForGrid()
{    
    if (robot == "icubSim")
        return true;

    imageL=new ImageOf<PixelRgb>;
    imageR=new ImageOf<PixelRgb>;

    Stamp TSLeft;
    Stamp TSRight;

    bool initL=false;
    bool initR=false;

    cv::Size boardSize;
    boardSize.width  = this->boardWidth;
    boardSize.height = this->boardHeight;
    /*cout << "boardSize : " << boardSize.width << "\t" << boardSize.height << endl;*/

    ImageOf<PixelRgb> *tmpL = imagePortInLeft->read(false);
    ImageOf<PixelRgb> *tmpR = imagePortInRight->read(false);

    if(tmpL!=NULL)
    {
        *imageL=*tmpL;
        imagePortInLeft->getEnvelope(TSLeft);
        initL=true;
    }
    if(tmpR!=NULL) 
    {
        *imageR=*tmpR;
        imagePortInRight->getEnvelope(TSRight);
        initR=true;
    }

    bool foundL=false;
    bool foundR=false;

	mutex->wait();
    if(initL && initR && checkTS(TSLeft.getTime(),TSRight.getTime()))
    {
        imgL = (IplImage*) imageL->getIplImage();
        imgR = (IplImage*) imageR->getIplImage();
        cv::Mat Left(imgL);
        cv::Mat Right(imgR);
        /*cout << "Left:  " << Left << endl;
        cout << "Right: " << Right << endl;*/

        std::vector<cv::Point2f> pointbufL;
        std::vector<cv::Point2f> pointbufR;
        foundL = findChessboardCorners( Left, boardSize, pointbufL,
                                        CV_CALIB_CB_ADAPTIVE_THRESH &
                                        CV_CALIB_CB_FAST_CHECK &
                                        CV_CALIB_CB_NORMALIZE_IMAGE);
        foundR = findChessboardCorners( Right, boardSize, pointbufR,
                                        CV_CALIB_CB_ADAPTIVE_THRESH &
                                        CV_CALIB_CB_FAST_CHECK &
                                        CV_CALIB_CB_NORMALIZE_IMAGE);
        printMessage(3,"Found (L & R): %i %i\n", foundL, foundR);
        if(foundL && foundR)
        {
            cv::Mat cL(pointbufL);
            cv::Mat cR(pointbufR);

            // save images
            if (currentWaypoint==1 && searchCounter == searchCounterMax-1)
            {
            	yarp::os::mkdir(path.c_str());
                saveStereoImage(path.c_str(),"START",imgL,imgR);
            }
            else if (currentWaypoint==numWaypoints && searchCounter == searchCounterMax-1)
            {
                saveStereoImage(path.c_str(),"END",imgL,imgR);
            }  

            if (currentWaypoint==1)
            {
                pointbufR_STRT = pointbufR;
                pointbufL_STRT = pointbufL;
            }
            else if (currentWaypoint==numWaypoints)
            {
                pointbufR_END = pointbufR;
                pointbufL_END = pointbufL;
                cv::Mat cL_STRT(pointbufL_STRT);
                cv::Mat cR_STRT(pointbufR_STRT);
                drawChessboardCorners(Left,  boardSize, cL_STRT, foundL);
                drawChessboardCorners(Right, boardSize, cR_STRT, foundR);
            }

            drawChessboardCorners(Left, boardSize, cL, foundL);
            drawChessboardCorners(Right, boardSize, cR, foundR);          
        }
        printMessage(3,"Writing images...\n");
        ImageOf<PixelRgb>& outimL=outPortLeft->prepare();
        outimL=*imageL;
        outPortLeft->write();

        ImageOf<PixelRgb>& outimR=outPortRight->prepare();
        outimR=*imageR;
        outPortRight->write();
    }
    else
    {
		printMessage(0,"TSLeft %g TSRIGHT %g\t",TSLeft.getTime(),TSRight.getTime());
		printMessage(0,"initL  %i initR   %i\n",initL,initR);
	}
    mutex->post();

    if (foundL && foundR)
    {
        searchCounter++;
        if (searchCounter == searchCounterMax)
        {
        	return true;
        }
    }
    else
        return false;

    delete imageL;
    delete imageR;

   return false;
}

void chessEyeTestThread::saveStereoImage(const string imageDir, const string type, IplImage* left, IplImage * right)
{
    string pathL = imageDir + "left_" + type + ".png";
    string pathR = imageDir + "right_" + type + ".png";

    cvSaveImage(pathL.c_str(),left);
    cvSaveImage(pathR.c_str(),right);

    printMessage(0,"Image saved to \n\t%s\n\t%s\n",pathL.c_str(),pathR.c_str());
}

bool chessEyeTestThread::goToWaypoint (const int &_cG)
{
    iposH->setRefSpeeds(jvWaypoints[_cG].vels.data());
    iposH->positionMove(jvWaypoints[_cG].jnts.data());
}

bool chessEyeTestThread::checkMotionDone()
{
	if (step != 2)
		return true;
	
    iencsH->getEncoders(encsH->data());
    Vector qH    = *encsH;
    Vector thres(qH.size(),0.0);

    bool done=1;
    for (int i = 0; i < qH.size(); i++)
    {
        thres[i] = fabs(qH[i]-jvWaypoints[currentWaypoint].jnts[i]);

        if (i == 4 || i == 5) // with the pan & tilt joints we can be way more precise
        	done = done && thres[i] < (epsilon/10.0);
        else
        	done = done && thres[i] < epsilon;
    }
    printMessage(2,"fabs(qH-jvWaypoints[%i].jnts) = %s\n",currentWaypoint,thres.toString().c_str());

    if (done)
    {
	    if (currentWaypoint == 0)
	    {
	    	encsH_STRT = qH;
	    	printMessage(1,"Saving encoders start: %s\n",encsH_STRT.toString().c_str());
			timeflag = 0;
    		return true;
	    }
	    else if ((currentWaypoint == numWaypoints-1) && (qH[0]-encsH_STRT[0] == 0.0))
	    {
	    	encsH_END = qH;
			printMessage(1,"Saving encoders end: %s\n",encsH_END.toString().c_str());
			timeflag = 0;
    		return true;
	    }
    }

    // if we're not at the end or the beginning, test for the timeout
    if (currentWaypoint != 0 && currentWaypoint != numWaypoints-1)
    {
    	if (timeflag == 0)
    	{
    		timeflag = 1;
    		timenow  = Time::now();
    	}
    	printMessage(1,"Time diff %g\n",Time::now()-timenow);
    	if (Time::now()-timenow > timeout)
    	{
    		printMessage(0,"TIMEOUT!!!\n");
    		timeflag = 0;
    		return true;
    	}
    }

    return false;
}

void chessEyeTestThread::delay(int sec)
{
    for (int i = 0; i < sec*4; ++i)
    {
        Time::delay(0.25);
    }
}

bool chessEyeTestThread::testAchievement()
{
    return true;
}

bool chessEyeTestThread::alignJointsBounds()
{
    double min = 0;
    double max = 0;

    printMessage(0,"numWaipoints: %i\n",numWaypoints);

    for (int i = 0; i < numWaypoints; i++)
    {
        for (int j = 0; j < jvWaypoints[i].jnts.size(); j++)
        {
  			// printMessage(0,"i j : %i %i\n",i,j);
            ilimH->getLimits(j,&min,&max);
            if (jvWaypoints[i].jnts[j] < min)
            {
                jvWaypoints[i].jnts[j] = min+DIST_FROM_THRES;
                printMessage(0,"WARNING: joints #%i of group #%i was out of the lower bound.\n",j,i);
                printMessage(0,"         set it to the joint's min limit, i.e. %g.\n",jvWaypoints[i].jnts[j]);
            }
            
            if (jvWaypoints[i].jnts[j] > max)
            {
                jvWaypoints[i].jnts[j] = max-DIST_FROM_THRES;
                printMessage(0,"WARNING: joints #%i of group #%i was out of the higher bound.\n",j,i);
                printMessage(0,"         set it to the joint's max limit, i.e. %g.\n",jvWaypoints[i].jnts[j]);
            }
        }
    }

    return true;
}

int chessEyeTestThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"*** %s: ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void chessEyeTestThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port -> interrupt();
        _port -> close();

        delete _port;
        _port = 0;
    }
}

void chessEyeTestThread::threadRelease()
{
    printMessage(0,"Returning to position mode..\n");
        goToWaypoint(0);

    printMessage(0,"Closing ports..\n");
        closePort(imagePortInLeft);
        closePort(imagePortInRight);
        printMessage(1,"input ports successfully closed!\n");
        closePort(outPortLeft);
        closePort(outPortRight);
        printMessage(1,"input ports successfully closed!\n");
        closePort(disparityPortIn);
        closePort(disparityPortOut);
        printMessage(1,"disparity ports successfully closed!\n");


    printMessage(0,"Closing controllers..\n");
        ddT.close();
        ddH.close();

    if (encsH)
    {
    	delete encsH;
    	encsH = NULL;
    }
    
    if (mutex)
    {
    	delete mutex;
    	mutex = NULL;
    }
}

// empty line to make gcc happy
