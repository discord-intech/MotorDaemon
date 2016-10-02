//
// Created by discord on 26/09/16.
//

#include "../include/MotionController.hpp"

bool MotionController::started;

MotionController::MotionController() : leftMotor(Side::LEFT), rightMotor(Side::RIGHT), direction(0, LOW_ANGLE, 100, HIGH_ANGLE), //FIXME bounds
rightSpeedPID(&currentRightSpeed, &rightPWM, &rightSpeedSetpoint),
leftSpeedPID(&currentLeftSpeed, &leftPWM, &leftSpeedSetpoint),
translationPID(&currentDistance, &translationSpeed, &translationSetpoint),
curvePID(&currentRadius, &radiusToSet, &curveSetpoint),
averageLeftSpeed(), averageRightSpeed(), odo(67,68,44,26) //TODO PINS odo
{
    translationSetpoint = 0;
    leftSpeedSetpoint = 0;
    rightSpeedSetpoint = 0;

    leftSpeedPID.setOutputLimits(-255,255);
    rightSpeedPID.setOutputLimits(-255,255);
    curvePID.setOutputLimits(DIST_MOTOR_DIRECTION/TAN(LOW_ANGLE), DIST_MOTOR_DIRECTION/TAN(HIGH_ANGLE));

    maxSpeed = 4000; // Vitesse maximum, des moteurs (avec une marge au cas o� on s'amuse � faire forcer un peu la bestiole).
    maxSpeedTranslation = 2000; // Consigne max envoy�e au PID
    maxAcceleration = 15;
    leftCurveRatio = 1;
    rightCurveRatio = 1;

    // maxjerk = 1; // Valeur de jerk maxi(secousse d'acc�l�ration)

    toleranceTranslation = 30;
    toleranceRotation = 50;
    toleranceSpeed = 50;
    toleranceSpeedEstablished = 50; // Doit �tre la plus petite possible, sans bloquer les trajectoires courbes 50
    delayToEstablish = 1000;


    toleranceDifferentielle = 500; // Pour les trajectoires "normales", v�rifie que les roues ne font pas nawak chacunes de leur cot�.

    translationPID.setTunings(1, 0, 0);
    leftSpeedPID.setTunings(0.01, 0.000025, 0.0001); // ki 0.00001
    rightSpeedPID.setTunings(0.01, 0.000025, 0.0001);
    curvePID.setTunings(1, 0, 0);

    distanceTest = 200;
}

void MotionController::init()
{
    leftMotor.initPWM();
    rightMotor.initPWM();
    direction.initPWM();

    started = true;

    t = std::thread(std::bind(mainWorker, this), "Feedback Thread");
    t.detach();
}

void MotionController::mainWorker(MotionController *asser)
{
    while(started)
    {
        asser->control();

        usleep((__useconds_t) (1000000. / FREQ_ASSERV));
    }
}

void MotionController::control()
{
    // Pour le calcul de la vitesse instantan�e :
    static long previousLeftTicks = 0;
    static long previousRightTicks = 0;

    // Pour le calcul de l'acc�l�ration intantan�e :
    static long previousLeftSpeedSetpoint = 0;
    static long previousRightSpeedSetpoint = 0;

    /*
    // Pour le calcul du jerk :
    static int32_t previousLeftAcceleration = 0;
    static int32_t previousRightAcceleration = 0;
    */

    long rightTicks = odo.getRightValue();

    long leftTicks = odo.getLeftValue();

    currentLeftSpeed = (leftTicks - previousLeftTicks)*FREQ_ASSERV; // (nb-de-tick-passés)*(freq_asserv) (ticks/sec)
    currentRightSpeed = (rightTicks - previousRightTicks)*FREQ_ASSERV;


    previousLeftTicks = leftTicks;
    previousRightTicks = rightTicks;

    averageLeftSpeed.add(currentLeftSpeed);
    averageRightSpeed.add(currentRightSpeed);

    currentLeftSpeed = averageLeftSpeed.value(); // On utilise pour l'asserv la valeur moyenne des dernieres current Speed
    currentRightSpeed = averageRightSpeed.value(); // sinon le robot il fait nawak.

    if(ABS(currentRightSpeed - currentLeftSpeed) > 0)
    {
        currentRadius = (volatile long) ((currentLeftSpeed * RAYON_COD_DROITE + currentRightSpeed * RAYON_COD_GAUCHE)
                                         / (TICK_TO_MM * (currentRightSpeed - currentLeftSpeed)));
    }
    else
    {
        currentRadius = INT64_MAX;
    }

    //TODO approx circulaire

    currentDistance = (leftTicks + rightTicks) / 2;
    currentAngle = ((rightTicks - currentDistance)*RAYON_COD_GAUCHE/RAYON_COD_DROITE - (leftTicks - currentDistance)) / 2;


    translationPID.compute();	// Actualise la valeur de 'translationSpeed'
/*    curvePID.compute();

    leftCurveRatio = (ABS(radiusToSet)-(RAYON_COD_GAUCHE*(radiusToSet<0?-1:1)))/(ABS(radiusToSet)+RAYON_COD_DROITE-RAYON_COD_GAUCHE);
    rightCurveRatio = (ABS(radiusToSet)+(RAYON_COD_DROITE*(radiusToSet<0?-1:1)))/(ABS(radiusToSet)+RAYON_COD_DROITE-RAYON_COD_GAUCHE);

    if(MAX(leftCurveRatio, rightCurveRatio) > 1.0)
    {
        float offset = (float) (1.0 - MAX(leftCurveRatio, rightCurveRatio));
        leftCurveRatio = MAX(leftCurveRatio+offset,0);
        rightCurveRatio = MAX(rightCurveRatio+offset,0);
    }

    if(leftCurveRatio<0)
        leftCurveRatio=0;
    if(rightCurveRatio<0)
        rightCurveRatio=0;
*/

    // Limitation de la consigne de vitesse en translation
    if(translationSpeed > maxSpeedTranslation)
        translationSpeed = maxSpeedTranslation;
    else if(translationSpeed < -maxSpeedTranslation)
        translationSpeed = -maxSpeedTranslation;


    leftSpeedSetpoint = (long) (translationSpeed * leftCurveRatio);
    rightSpeedSetpoint = (long) (translationSpeed * rightCurveRatio);

    // Limitation de la vitesse
    if(leftSpeedSetpoint > maxSpeed)
        leftSpeedSetpoint = maxSpeed;
    else if(leftSpeedSetpoint < -maxSpeed)
        leftSpeedSetpoint = -maxSpeed;
    if(rightSpeedSetpoint > maxSpeed)
        rightSpeedSetpoint = maxSpeed;
    else if(rightSpeedSetpoint < -maxSpeed)
        rightSpeedSetpoint = -maxSpeed;
    ;

    // Limitation de l'accélération du moteur gauche (permet de règler la pente du trapèze de vitesse)
    if(leftSpeedSetpoint - previousLeftSpeedSetpoint > maxAcceleration)
    {
        leftSpeedSetpoint = previousLeftSpeedSetpoint + maxAcceleration*leftCurveRatio;
    }
    else if(leftSpeedSetpoint - previousLeftSpeedSetpoint < -maxAcceleration)
    {
        leftSpeedSetpoint = previousLeftSpeedSetpoint - maxAcceleration*leftCurveRatio;
    }

    // Limitation de l'acc�l�ration du moteur droit
    if(rightSpeedSetpoint - previousRightSpeedSetpoint > maxAcceleration)
    {
        rightSpeedSetpoint = previousRightSpeedSetpoint + maxAcceleration*rightCurveRatio;
    }
    else if(rightSpeedSetpoint - previousRightSpeedSetpoint < -maxAcceleration)
    {
        rightSpeedSetpoint = previousRightSpeedSetpoint - maxAcceleration*rightCurveRatio;
    }



    previousLeftSpeedSetpoint = leftSpeedSetpoint;			// Mise à jour des consignes de vitesse
    previousRightSpeedSetpoint = rightSpeedSetpoint;



    leftSpeedPID.compute();		// Actualise la valeur de 'leftPWM'
    rightSpeedPID.compute();	// Actualise la valeur de 'rightPWM'

    leftMotor.run(leftPWM);
    rightMotor.run(rightPWM);

    //direction.setAngle(ARCTAN(DIST_MOTOR_DIRECTION/radiusToSet));
}

void MotionController::stop()
{

    translationSetpoint = currentDistance;
    leftSpeedSetpoint = 0;
    rightSpeedSetpoint = 0;

    leftMotor.run(0);
    rightMotor.run(0);
    leftCurveRatio = 1.0;
    rightCurveRatio = 1.0;
    translationPID.resetErrors();
    leftSpeedPID.resetErrors();
    rightSpeedPID.resetErrors();

}

void MotionController::setTranslationTunings(float kp, float ki, float kd)
{
    translationPID.setTunings(kp, ki, kd);
}

void MotionController::setLeftSpeedTunings(float kp, float ki, float kd)
{
    leftSpeedPID.setTunings(kp, ki, kd);
}

void MotionController::setRightSpeedTunings(float kp, float ki, float kd)
{
    rightSpeedPID.setTunings(kp, ki, kd);
}

void MotionController::testPosition()
{
    orderTranslation(distanceTest);
}

void MotionController::orderTranslation(long mmDistance)
{
    translationSetpoint += (int32_t) mmDistance / TICK_TO_MM;
}

Odometry* MotionController::getOdometry(void)
{
    return &odo;
}

long MotionController::getCurveRadius()
{
    return currentRadius;
}
