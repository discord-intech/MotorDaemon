//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_MOTIONCONTROLLER_HPP
#define MOTORDAEMON_MOTIONCONTROLLER_HPP

#include <sys/time.h>
#include <chrono>
#include "Motor.hpp"
#include "pid.hpp"
#include "average.hpp"
#include "Odometry.hpp"
#include "Servo.hpp"

#define AVERAGE_SPEED_SIZE	15

#define FREQ_ASSERV 1300

#define PI 3.14159

#define RAYON_COD_GAUCHE 35
#define RAYON_COD_DROITE 35

#define DIST_MOTOR_DIRECTION 160

#define LOW_ANGLE -0.58
#define HIGH_ANGLE 0.58  //TODO Bounds

#define DELTA_FREQ_REFRESH 500

#define MM_PER_TICK 0.4331

//#define MILLIS() std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count()

class MotionController
{
private:

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
    std::shared_ptr<long> curveSetpoint = std::shared_ptr<long>(new long(0));
    std::shared_ptr<long> currentRadius = std::shared_ptr<long>(new long(0));
    std::shared_ptr<long> radiusToSet = std::shared_ptr<long>(new long(0));

    //	Limitation de vitesses
    long maxSpeed; 				// definit la vitesse maximal des moteurs du robot
    long maxSpeedTranslation;	// definit la consigne max de vitesse de translation envoi�e au PID (trap�ze)

    //	Limitation d'acc�l�ration
    long maxAcceleration;

    long currentAngle;			// ticks

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
    long toleranceRotation;

    long toleranceSpeed; // Tol�rance avant de consid�rer le mouvement anormal (�cart entre la consigne de vitesse et la vitesse r�elle)
    long toleranceSpeedEstablished; // Tol�rance autour de la vitesse �tablie avant de capter un blocage

    long toleranceDifferentielle;

    long delayToEstablish; // Temps � attendre avant de consid�rer la vitesse stable

    int32_t distanceTest;

    bool moving = false;

    std::thread t;

    unsigned int delayToStop;  //En ms

    static void mainWorker(MotionController*&);



public:
    void control(void);

    MotionController(void);
    void init(void);

    void stop(void);

    void updatePosition(void);
    void manageStop(void);

    void orderTranslation(long);
    void orderAngle(float);

    void setTranslationTunings(float, float, float);
    void setLeftSpeedTunings(float, float, float);
    void setRightSpeedTunings(float, float, float);

    void testPosition(void);

    void testSpeed(bool);

    bool isPhysicallyStopped(void);

    long getTranslationSetPoint(void) { return *translationSetpoint;}

    Odometry* getOdometry(void);
    long getCurveRadius(void);

    void printTranslationError(void)
    {
        std::cout << "Trans err : " << *translationSetpoint - *currentDistance << " ; "
                  << translationPID.getError() << " ; " << *leftSpeedSetpoint << std::endl;
    }

};


#endif //MOTORDAEMON_MOTIONCONTROLLER_HPP
