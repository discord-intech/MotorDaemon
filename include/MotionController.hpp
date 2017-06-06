//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_MOTIONCONTROLLER_HPP
#define MOTORDAEMON_MOTIONCONTROLLER_HPP

#include <sys/time.h>
#include <chrono>
#include <vector>
#include <math.h>
#include <queue>
#include "Motor.hpp"
#include "pid.hpp"
#include "average.hpp"
#include "Odometry.hpp"
#include "Servo.hpp"
#include "Settings.hpp"
#include "Cinematic.hpp"
#include "Selector.hpp"

#define AVERAGE_SPEED_SIZE	25

#define FREQ_ASSERV 1300

#define PI 3.14159

#define RAYON_COD_GAUCHE 115
#define RAYON_COD_DROITE 115

#define MAX_RADIUS 10000

#define DIST_MOTOR_DIRECTION 150

#define LOW_ANGLE -0.79
#define HIGH_ANGLE 0.79 //TODO Bounds

#define DELTA_FREQ_REFRESH 500

#define GO_COUNTER_THRESHOLD 1000

#define MM_PER_TICK 1.4211
#define TICKS_TO_RAD 0.00189

//#define MILLIS() std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count()



class MotionController
{
private:

    Settings settings;

    LeftMotor leftMotor;
    RightMotor rightMotor;
    Odometry odo;
    Servo direction;

    //	Asservissement en vitesse du moteur droit
    PID rightSpeedPID;
    std::shared_ptr<long> rightSpeedSetpoint = std::shared_ptr<long>(new long(0));	// ticks/seconde
    std::shared_ptr<long> currentRightSpeed = std::shared_ptr<long>(new long(0));		// ticks/seconde
    std::shared_ptr<long> rightPWM = std::shared_ptr<long>(new long(0));

    //	Asservissement en vitesse du moteur gauche
    PID leftSpeedPID;
    std::shared_ptr<long> leftSpeedSetpoint = std::shared_ptr<long>(new long(0));		// ticks/seconde
    std::shared_ptr<long> currentLeftSpeed = std::shared_ptr<long>(new long(0));		// ticks/seconde
    std::shared_ptr<long> leftPWM = std::shared_ptr<long>(new long(0));

    //	Asservissement en position : translation
    PID translationPID;
    std::shared_ptr<long> translationSetpoint = std::shared_ptr<long>(new long(0));	// ticks
    std::shared_ptr<long> currentDistance = std::shared_ptr<long>(new long(0));		// ticks
    std::shared_ptr<long> translationSpeed = std::shared_ptr<long>(new long(0));		// ticks/seconde

    PID curvePID; //FIXME INIT
    std::shared_ptr<long> curveSetpoint = std::shared_ptr<long>(new long(INT32_MAX));
    std::shared_ptr<long> currentRadius = std::shared_ptr<long>(new long(INT32_MAX));
    std::shared_ptr<long> deltaRadius = std::shared_ptr<long>(new long(0));

    //	Limitation de vitesses
    volatile long maxSpeed; 				// definit la vitesse maximal des moteurs du robot
    volatile long maxSpeedTranslation;	// definit la consigne max de vitesse de translation envoi�e au PID (trap�ze)

    //	Limitation d'acc�l�ration
    long maxAcceleration;
    long maxDecceleration;

    std::shared_ptr<double> x = std::shared_ptr<double>(new double(0)); //mm
    std::shared_ptr<double> y = std::shared_ptr<double>(new double(0)); //mm
    std::shared_ptr<double> currentAngle = std::shared_ptr<double>(new double(0)); //rads
    std::shared_ptr<double> originAngle = std::shared_ptr<double>(new double(0)); //rads

    std::queue<Cinematic> pointsToPass = std::queue<Cinematic>();

    //Les ratios de vitesse pour commander un d�placement courbe
    double leftCurveRatio;
    double rightCurveRatio;

    static bool started;

    static long startTime;
    static long execTime;

    Average<long, AVERAGE_SPEED_SIZE> averageLeftSpeed;
    Average<long, AVERAGE_SPEED_SIZE> averageRightSpeed;

    //Nombre de ticks de tol�rance pour consid�rer qu'on est arriv� � destination
    long toleranceTranslation;

    long toleranceSpeed; // Tol�rance avant de consid�rer le mouvement anormal (�cart entre la consigne de vitesse et la vitesse r�elle)
    long toleranceSpeedEstablished; // Tol�rance autour de la vitesse �tablie avant de capter un blocage

    long toleranceDifferentielle;

    long delayToEstablish; // Temps � attendre avant de consid�rer la vitesse stable

    int32_t distanceTest;

    volatile bool moving = false;

    volatile bool controlled = true;

    volatile bool sweeping = false;

    volatile bool lastWay = true;

    volatile bool servoMotor = true;

    volatile int GOcounter = 0;

    volatile long sweepRadius = 10000;

    std::thread t;

    unsigned int delayToStop;  //En ms

    std::fstream outPos;

    float direction_table[MAX_RADIUS];

    void compute_direction_table(void);

    static void mainWorker(MotionController*&);



public:

    std::shared_ptr<bool> stahp = std::shared_ptr<bool>(new bool(false));

    void control(void);

    MotionController(Settings&);
    void init(void);

    void stop(void);

    void updatePosition(void);
    void manageStop(void);

    void orderTranslation(long);
    void orderAngle(float);

    void setSpeedTranslation(int);

    void orderCurveRadius(long);

    void setTranslationTunings(float, float, float);
    void setCurveTunings(float, float, float);
    void setLeftSpeedTunings(float, float, float);
    void setRightSpeedTunings(float, float, float);

    void setPosition(double xn, double yn) { *x = xn; *y = yn; }
    void setAngle(double o) { *originAngle = o - *currentAngle; }

    const char* getTunings(void);

    void testPosition(void);

    void testSpeed(int);

    void setTrajectory(std::vector<Cinematic>&, long);

    const char* isMoving(void) { return std::string(std::string(moving ? "1" : "0")+std::string("\r\n")).c_str(); }

    bool isPhysicallyStopped(void);

    long getTranslationSetPoint(void) { return *translationSetpoint; }

    void go(void) { *translationSpeed = maxSpeedTranslation; GOcounter = 1; }

    void goR(void) { *translationSpeed = (long)(-1.0 * maxSpeedTranslation * 0.60); GOcounter = 1; }

    void setControlled(bool b) { controlled = b; }

    void sweep(bool way);

    void stopSweep(void);

    Odometry* getOdometry(void);
    long getCurveRadius(void);

    double getX(void) {return *x;}

    double getY(void) {return *y;}

    long getSpeed(void) {return maxSpeedTranslation;}

    long getSpeedL(void) { return *currentLeftSpeed; }

    long getSpeedR(void) { return *currentRightSpeed; }

    long getCSpeedL(void) { return *leftSpeedSetpoint; }

    long getCSpeedR(void)  { return *rightSpeedSetpoint; }

    double getAngle(void) { return *currentAngle;}

    void loadPos();

    void printTranslationError(void)
    {
        std::cout << "Trans err : " << *translationSetpoint - *currentDistance << " ; "
                  << translationPID.getError() << " ; " << *leftSpeedSetpoint << std::endl;
    }

};


#endif //MOTORDAEMON_MOTIONCONTROLLER_HPP
