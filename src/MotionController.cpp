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
    leftSpeedPID.setTunings(1, 0, 0); // ki 0.00001
    rightSpeedPID.setTunings(1, 0, 0);
    curvePID.setTunings(1, 0, 0);

    distanceTest = 200;

    delayToStop = 100;
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
        asser->manageStop();
        asser->updatePosition();

        usleep((__useconds_t) (1000000 / FREQ_ASSERV));
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

void MotionController::manageStop()
{
    static uint32_t time = 0;
  //  static uint32_t time2 = 0;
  //  static uint32_t time3 = 0;
  //  static uint32_t timeToEstablish = 0;
  //  static uint32_t timeNotEstablished = 0;
  //  static bool isSpeedEstablished = false;

    if (isPhysicallyStopped() && moving) // Pour un blocage classique
    {

        if (time == 0)
        { //D�but du timer
            time = MILLIS();
        }
        else
        {
            if ((MILLIS() - time) >= delayToStop)
            { //Si arr�t� plus de 'delayToStop' ms
                if (ABS(translationPID.getError()) <= toleranceTranslation)
                { //Stop� pour cause de fin de mouvement
                    stop();
                   // moveAbnormal = false;
                }
                else
                { //Stopp� pour blocage
                  //  stop();
                  //  moveAbnormal = true;
                }
            }
        }
    }

    /*else if(moving && !isSpeedEstablished && !forcedMovement && curveMovement){ // V�rifie que le ratio reste bon pdt les traj courbes

        if (leftCurveRatio<rightCurveRatio && averageRightSpeed.value() !=0 && rightCurveRatio!=0){ // si on tourne a gauche
            if (ABS((averageLeftSpeed.value()/averageRightSpeed.value())-(leftCurveRatio/rightCurveRatio))>toleranceCurveRatio){
                stop();
                moveAbnormal = true;
            }
        }
        else if(rightCurveRatio<leftCurveRatio && averageLeftSpeed.value()!=0 && leftCurveRatio!=0){ //si on tourne � droite
            if (ABS((averageRightSpeed.value()/averageLeftSpeed.value())-(rightCurveRatio/leftCurveRatio))>toleranceCurveRatio){
                stop();
                moveAbnormal = true;
            }
        }
    }*/ //SOULD NOT BE USEFUL


 /*   else if ((isLeftWheelSpeedAbnormal() || isRightWheelSpeedAbnormal()) && curveMovement && !forcedMovement) // Sert a v�rifier que les consignes de vitesse sont bien respect�es (blocage pour les trajectoires courbes)
    {
        if (time2 == 0)
        { //D�but du timer
            time2 = Millis();
        }
        else
        {
            if (ABS(translationPID.getError()) <= toleranceTranslation && ABS(rotationPID.getError()) <= toleranceRotation)
            { //Stopp� pour cause de fin de mouvement
                stop();
                isSpeedEstablished = false;
                moveAbnormal = false;
            }
            else if (((Millis() - time2) >= delayToStopCurve) && isSpeedEstablished){

                stop();
                isSpeedEstablished = false;
                moveAbnormal = true;

            }
        }
    }*/

  /*  else if (forcedMovement && moving){

        if (time3 == 0)
        {
            time3 = Millis();
        }
        else
        {
            if ((Millis() - time3) >= delayToStop){
                if (ABS(translationPID.getError()) <= toleranceTranslation && ABS(rotationPID.getError()) <= toleranceRotation)
                { //Stopp� pour cause de fin de mouvement
                    stop();
                    moveAbnormal = false;

                }
            }
        }
    }*/



    else
    {
        time = 0;
    //    time2 =0;
     //   time3 = 0; // Test
    }
}

void MotionController::updatePosition() {
    //TODO approx circulaire
}

bool MotionController::isPhysicallyStopped() {
    return (translationPID.getDerivativeError() == 0) || (ABS(ABS(leftSpeedPID.getError())-ABS(rightSpeedPID.getError()))>toleranceDifferentielle);
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
    if(!moving)
    {
        translationPID.resetErrors();
        moving = true;
    }
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
