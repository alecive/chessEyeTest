/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <yarp/os/Semaphore.h>

#include <gsl/gsl_math.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <deque>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;

using namespace std;

string int_to_string( const int a );

struct jointsVelocities
{
    string name;
    Vector jnts;       // head joints
    Vector vels;       // head velocities

    jointsVelocities();
    jointsVelocities(string _name, Vector _jnts, Vector _vels);

    /**
    * Copy Operator
    **/
    jointsVelocities &operator=(const jointsVelocities &jv);

    /**
    * Print function
    **/
    void print();
};

class chessEyeTestThread: public RateThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // 
    double epsilon;
    // 
    double timeout;
    //
    int type;
    //
    int iterations;
    //
    string inputL;
    string inputR;
    string inputDisp;
    string textToPrint;
    //
    string path;
    //
    string configfile;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    // CALIB THINGS:
        // TEST = 0
        int   boardWidth;
        int  boardHeight;

        BufferedPort<ImageOf<PixelRgb> > *imagePortInLeft;
        BufferedPort<ImageOf<PixelRgb> > *imagePortInRight;
        BufferedPort<ImageOf<PixelRgb> > *outPortRight;
        BufferedPort<ImageOf<PixelRgb> > *outPortLeft;

        ImageOf<PixelRgb> *imageL;
        ImageOf<PixelRgb> *imageR;
        IplImage * imgL;
        IplImage * imgR;

        int searchCounter;
        int searchCounterMax;

        std::vector<cv::Point2f> pointbufL_STRT;
        std::vector<cv::Point2f> pointbufR_STRT;
        std::vector<cv::Point2f> pointbufL_END;
        std::vector<cv::Point2f> pointbufR_END;

        // TEST = 1
        BufferedPort<ImageOf<PixelBgr> > *disparityPortIn;
        BufferedPort<ImageOf<PixelBgr> > *disparityPortOut;

        ImageOf<PixelBgr> *imageDispIn;

        cv::Mat bwDispIn_STRT;
        cv::Mat bwDispIn_END;


    Semaphore *mutex;

    // BufferedPort<Bottle> *outPort;
    deque<jointsVelocities> jvWaypoints;
    int currentWaypoint;
    int numWaypoints;

    int  step;    // Flag to know in which step the thread is

    // Driver for "classical" interfaces
    PolyDriver       ddH; // right arm device driver
    PolyDriver       ddT; // left arm  device driver

    // "Classical" interfaces - TORSO
    IEncoders         *iencsT;
    IPositionControl  *iposT;
    IControlLimits    *ilimT;
    Vector            *encsT;
    int jntsT;

    // "Classical" interfaces - HEAD
    IEncoders         *iencsH;
    IPositionControl  *iposH;
    IControlLimits    *ilimH;
    Vector            *encsH;
    int jntsH;

    Vector encsH_STRT;
    Vector encsH_END;

    int timeflag;
    double timenow;

    /**
    * Aligns joint bounds according to the actual limits of the robot
    */
    bool alignJointsBounds();

    /**
    * Checks if the motion has finished. To be implemented in future releases
    * (the old version is not available any more because of the changes)
    */
    bool checkMotionDone();

    /**
    * Creates a delay (what for? In order to remember
    * that I have to improve my module and remove this!)
    */
    void delay(int sec);

    /**
    * Moves the head to the desired waypoint configuration
    */
    bool goToWaypoint (const int &_cG);
    
    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

    /**
     * Closes properly a given port
    **/
    void closePort(yarp::os::Contactable *_port);

    /**
    * Verify the degree of achievements of the task, by reading joints encoders
    */
    bool testAchievement();

    /**
    * Save stereo images for the chessboard
    */
    void saveStereoImage(const string imageDir, const string type, IplImage* left, IplImage * right);

    /**
    * It's a function pointer; it points to a specific call according to the test
    * being executed.
    **/
    bool (chessEyeTestThread::*testUnderExecution)();

    /**
    * Compares the disparity before and after the movement
    **/
    bool compareDisparity();

    /**
    * Searches for the grid
    **/
    bool searchForGrid();

    /**
    * Checks the timestamps
    **/
    bool checkTS(double TSLeft, double TSRight, double th=0.08);

    /**
    * Prints results
    **/
    void printResults();

public:
    // CONSTRUCTOR
    chessEyeTestThread(int _rate, string _name, string _robot, int _v, int _nG, double _eps, double _tO, const ResourceFinder &_rf);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
};

// PERSONAL NOTES (remember to remove them sooner or later!)
// iCub/main/src/tools/skinManagerGui
// iCub/main/src/modules/skinManager
// iCub/main/src/libraries/skinDynLib
//
// A skinContactList is represented as a list of lists
// where each list is a skinContact
// basically, it's a vector of skinContact
//
// A skinContact is a list of 8 elements that are:
// - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
// - a list of 3 double, i.e. the CoP
// - a list of 3 double, i.e. the force
// - a list of 3 double, i.e. the moment
// - a list of 3 double, i.e. the geometric center
// - a list of 3 double, i.e. the normal direction
// - a list of N int, i.e. the active taxel ids
// - a double, i.e. the pressure
// 
// ((48725 4 4 5) (-0.017 0.062 -0.036) (0.476424 0.109944 0.611614)
// (0.0 0.0 0.0) (-0.017 0.062 -0.036) (-0.585 -0.135 -0.751) (134) 16.288001)

