//
// Created by discord on 26/09/16.
//

#ifndef MOTORDAEMON_MOTIONCONTROLLER_HPP
#define MOTORDAEMON_MOTIONCONTROLLER_HPP

#include <chrono>
#include "Motor.hpp"
#include "pid.hpp"
#include "average.hpp"
#include "Odometry.hpp"
#include "Servo.hpp"

#define AVERAGE_SPEED_SIZE	25

#define FREQ_ASSERV 2000

#define PI 3.14159265

#define RAYON_COD_GAUCHE 35
#define RAYON_COD_DROITE 35

#define DIST_MOTOR_DIRECTION 160

#define LOW_ANGLE -0.58
#define HIGH_ANGLE 0.58  //TODO Bounds

#define TICK_TO_MM 0.04908

#define MILLIS() std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count()

class MotionController
{
private:

    Motor leftMotor;
    Motor rightMotor;
    Odometry odo;
    Servo direction;

    //	Asservissement en vitesse du moteur droit
    PID rightSpeedPID;
    volatile long rightSpeedSetpoint;	// ticks/seconde
    volatile long currentRightSpeed;		// ticks/seconde
    volatile long rightPWM;

    //	Asservissement en vitesse du moteur gauche
    PID leftSpeedPID;
    volatile long leftSpeedSetpoint;		// ticks/seconde
    volatile long currentLeftSpeed;		// ticks/seconde
    volatile long leftPWM;

    //	Asservissement en position : translation
    PID translationPID;
    volatile long translationSetpoint;	// ticks
    volatile long currentDistance;		// ticks
    volatile long translationSpeed;		// ticks/seconde

    PID curvePID;
    volatile long curveSetpoint;
    volatile long currentRadius;
    volatile long radiusToSet;

    //	Limitation de vitesses
    volatile long maxSpeed; 				// definit la vitesse maximal des moteurs du robot
    volatile long maxSpeedTranslation;	// definit la consigne max de vitesse de translation envoi�e au PID (trap�ze)

    //	Limitation d'acc�l�ration
    volatile long maxAcceleration;

    volatile long currentAngle;			// ticks

    //Les ratios de vitesse pour commander un d�placement courbe
    volatile float leftCurveRatio;
    volatile float rightCurveRatio;

    static bool started;

    Average<long, AVERAGE_SPEED_SIZE> averageLeftSpeed;
    Average<long, AVERAGE_SPEED_SIZE> averageRightSpeed;

    //Nombre de ticks de tol�rance pour consid�rer qu'on est arriv� � destination
    int toleranceTranslation;
    int toleranceRotation;

    int toleranceSpeed; // Tol�rance avant de consid�rer le mouvement anormal (�cart entre la consigne de vitesse et la vitesse r�elle)
    int toleranceSpeedEstablished; // Tol�rance autour de la vitesse �tablie avant de capter un blocage

    int toleranceDifferentielle;

    int delayToEstablish; // Temps � attendre avant de consid�rer la vitesse stable

    int32_t distanceTest;

    bool moving = false;

    std::thread t;

    unsigned int delayToStop;  //En ms


public:
    void control(void);

    MotionController(void);
    void init(void);

    static void mainWorker(MotionController*);

    void stop(void);

    void updatePosition(void);
    void manageStop(void);

    void orderTranslation(long);

    void setTranslationTunings(float, float, float);
    void setLeftSpeedTunings(float, float, float);
    void setRightSpeedTunings(float, float, float);

    void testPosition(void);

    bool isPhysicallyStopped(void);

    Odometry* getOdometry(void);
    long getCurveRadius(void);

};


#endif //MOTORDAEMON_MOTIONCONTROLLER_HPP
