//
// Created by discord on 26/09/16.
//

#include "../include/MotionController.hpp"
#include <sched.h>
#include <sys/resource.h>

bool MotionController::started;
long MotionController::startTime;
long MotionController::execTime;

unsigned long Millis(void)
{
    struct timeval tv;
    if(gettimeofday(&tv, NULL) != 0) return 0;
    return (tv.tv_sec * 1000ul) + (tv.tv_usec / 1000ul);
}

unsigned long Micros(void)
{
    struct timeval tv;
    if(gettimeofday(&tv, NULL) != 0) return 0;
    return (unsigned long) (1000000 * tv.tv_sec + tv.tv_usec);
}

MotionController::MotionController() :  rightMotor(), leftMotor(), direction(1100000, LOW_ANGLE, 1550000, HIGH_ANGLE), //FIXME bounds
rightSpeedPID(), leftSpeedPID(), translationPID(), curvePID(),
averageLeftSpeed(), averageRightSpeed(), odo(67,68,44,26)
{
    execTime = 0;
    startTime = 0;

    rightSpeedPID.setPointers(currentRightSpeed, rightPWM, rightSpeedSetpoint);
    leftSpeedPID.setPointers(currentLeftSpeed, leftPWM, leftSpeedSetpoint);
    translationPID.setPointers(currentDistance, translationSpeed, translationSetpoint);
    curvePID.setPointers(currentRadius, deltaRadius, curveSetpoint);

    leftSpeedPID.setOutputLimits(-255,255);
    rightSpeedPID.setOutputLimits(-255,255);
    curvePID.setOutputLimits(DIST_MOTOR_DIRECTION/TAN(LOW_ANGLE), DIST_MOTOR_DIRECTION/TAN(HIGH_ANGLE));

    leftSpeedPID.setEpsilon(20);
    rightSpeedPID.setEpsilon(20);

    maxSpeed = 5000; // Vitesse maximum, des moteurs (avec une marge au cas o� on s'amuse � faire forcer un peu la bestiole).
    maxSpeedTranslation = 4000; // Consigne max envoy�e au PID
    maxAcceleration = 1000;
    leftCurveRatio = 1;
    rightCurveRatio = 1;

    // maxjerk = 1; // Valeur de jerk maxi(secousse d'acc�l�ration)

    toleranceTranslation = 30;
    toleranceRotation = 50;
    toleranceSpeed = 50;
    toleranceSpeedEstablished = 50; // Doit �tre la plus petite possible, sans bloquer les trajectoires courbes 50
    delayToEstablish = 1000;


    toleranceDifferentielle = 500; // Pour les trajectoires "normales", v�rifie que les roues ne font pas nawak chacunes de leur cot�.

    translationPID.setTunings(19, 0.001, 0);
    leftSpeedPID.setTunings(0.1, 0.00001, 0.0001); // ki 0.00001
    rightSpeedPID.setTunings(0.1, 0.00001, 0.0001);
    curvePID.setTunings(0, 0, 0);

    distanceTest = 200;

    delayToStop = 100;
}

void MotionController::init()
{
    leftMotor.initPWM();
    rightMotor.initPWM();
    direction.initPWM();

    direction.setAngle(0);

    started = true;

    t = std::thread(std::bind(mainWorker, this), "Feedback Thread");
    t.detach();
}

void MotionController::mainWorker(MotionController *&asser)
{
    int count=0;
    long lastTime = Millis();

  /*  sched_param par;
    par.__sched_priority=sched_get_priority_max(SCHED_RR);
    if(sched_setscheduler(0, SCHED_RR, &par) != 0)
    {
        std::cerr << "Failed to set RR sched !!" << std::endl;
        return;
    }*/

    while(started)
    {

        asser->control();

        count++;

        if(count % 5 == 0)
        {
            asser->manageStop();
            asser->updatePosition();
        }

        if(count == 10000)
        {
            count = 0;
            std::cout << "Frequency : " << 10000000./(double)(Millis() - lastTime) << " Hz" << std::endl;
           // asser->printTranslationError();
            lastTime = Millis();
        }

        execTime = Micros() - startTime;

        //usleep((__useconds_t) (1000000 / FREQ_ASSERV));
        timespec t, r;
        t.tv_sec=0;
        t.tv_nsec = (1000000000 / FREQ_ASSERV) - (execTime*1000);
        nanosleep(&t, &r);
    }
}

void MotionController::control()
{
    static long time = Millis();

    static long freq(0);

    static int counter(0);

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

 /*   if(freq == 0)
    {
        *currentLeftSpeed = (leftTicks - previousLeftTicks)*FREQ_ASSERV; // (nb-de-tick-passés)*(freq_asserv) (ticks/sec)
        *currentRightSpeed = (rightTicks - previousRightTicks)*FREQ_ASSERV;
    }
    else
    {
        *currentLeftSpeed = (leftTicks - previousLeftTicks)*freq; // (nb-de-tick-passés)*(freq_asserv) (ticks/sec)
        *currentRightSpeed = (rightTicks - previousRightTicks)*freq;
    }*/

    *currentLeftSpeed = (long) ((leftTicks - previousLeftTicks) / ((Micros()-startTime) / 1000000.)); // (nb-de-tick-passés)*(freq_asserv) (ticks/sec)
    *currentRightSpeed = (long) ((rightTicks - previousRightTicks) / ((Micros()-startTime) / 1000000.));

    startTime = Micros();

    previousLeftTicks = leftTicks;
    previousRightTicks = rightTicks;

    averageLeftSpeed.add(*currentLeftSpeed);
    averageRightSpeed.add(*currentRightSpeed);

    *currentLeftSpeed = averageLeftSpeed.value(); // On utilise pour l'asserv la valeur moyenne des dernieres current Speed
    *currentRightSpeed = averageRightSpeed.value(); // sinon le robot il fait nawak.

    if(ABS(*currentRightSpeed - *currentLeftSpeed) > 5)
    {
        *currentRadius = (long) ((*currentLeftSpeed * RAYON_COD_DROITE + *currentRightSpeed * RAYON_COD_GAUCHE)
                                         / (MM_PER_TICK * (*currentRightSpeed - *currentLeftSpeed)));
    }
    else
    {
        *currentRadius = INT64_MAX;
    }


    *currentDistance = (leftTicks + rightTicks) / 2;
    currentAngle = ((rightTicks - *currentDistance)*RAYON_COD_GAUCHE/RAYON_COD_DROITE - (leftTicks - *currentDistance)) / 2;


    translationPID.compute();

    curvePID.compute();


    if(ABS(*curveSetpoint + *deltaRadius) < MAX_RADIUS)
    {
        leftCurveRatio = ((double)ABS(*curveSetpoint + *deltaRadius)-(RAYON_COD_GAUCHE*(curveSetpoint<0?-1.0:1.0)))/((double)ABS(*curveSetpoint + *deltaRadius)+RAYON_COD_DROITE-RAYON_COD_GAUCHE);
        rightCurveRatio = ((double)ABS(*curveSetpoint + *deltaRadius)+(RAYON_COD_DROITE*(curveSetpoint<0?-1.0:1.0)))/((double)ABS(*curveSetpoint + *deltaRadius)+RAYON_COD_DROITE-RAYON_COD_GAUCHE);
    }
    else
    {
        leftCurveRatio = 1.0;
        rightCurveRatio = 1.0;
    }

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


    // Limitation de la consigne de vitesse en translation
    if(*translationSpeed > maxSpeedTranslation)
        *translationSpeed = maxSpeedTranslation;
    else if(*translationSpeed < -maxSpeedTranslation)
        *translationSpeed = -maxSpeedTranslation;


    *leftSpeedSetpoint = (long) (*translationSpeed * leftCurveRatio);
    *rightSpeedSetpoint = (long) (*translationSpeed * rightCurveRatio);

    // Limitation de la vitesse
    if(*leftSpeedSetpoint > maxSpeed)
        *leftSpeedSetpoint = maxSpeed;
    else if(*leftSpeedSetpoint < -maxSpeed)
        *leftSpeedSetpoint = -maxSpeed;
    if(*rightSpeedSetpoint > maxSpeed)
        *rightSpeedSetpoint = maxSpeed;
    else if(*rightSpeedSetpoint < -maxSpeed)
        *rightSpeedSetpoint = -maxSpeed;


    // Limitation de l'accélération du moteur gauche (permet de règler la pente du trapèze de vitesse)
    if(*leftSpeedSetpoint - previousLeftSpeedSetpoint > maxAcceleration)
    {
        *leftSpeedSetpoint = previousLeftSpeedSetpoint + maxAcceleration*leftCurveRatio;
    }
    else if(*leftSpeedSetpoint - previousLeftSpeedSetpoint < -maxAcceleration)
    {
        *leftSpeedSetpoint = previousLeftSpeedSetpoint - maxAcceleration*leftCurveRatio;
    }

    // Limitation de l'acc�l�ration du moteur droit
    if(*rightSpeedSetpoint - previousRightSpeedSetpoint > maxAcceleration)
    {
        *rightSpeedSetpoint = previousRightSpeedSetpoint + maxAcceleration*rightCurveRatio;
    }
    else if(*rightSpeedSetpoint - previousRightSpeedSetpoint < -maxAcceleration)
    {
        *rightSpeedSetpoint = previousRightSpeedSetpoint - maxAcceleration*rightCurveRatio;
    }



    previousLeftSpeedSetpoint = *leftSpeedSetpoint;			// Mise à jour des consignes de vitesse
    previousRightSpeedSetpoint = *rightSpeedSetpoint;



    leftSpeedPID.compute();		// Actualise la valeur de 'leftPWM'
    rightSpeedPID.compute();	// Actualise la valeur de 'rightPWM'

    //std::cout << "calculation time : " << Millis() - time << std::endl;
   // time = Millis();

    leftMotor.run((int) *leftPWM);
    rightMotor.run((int) *rightPWM);

    long t = Millis();

    if(t-time >= DELTA_FREQ_REFRESH)
    {
        //freq = counter / (t - time);
        time = t;
        counter = 0;
       // std::cout << "it's me : " << (long)translationPID.getPTR() << " : " <<(long)&currentDistance << " : " << currentDistance << " : " << translationSetpoint << " : " <<translationPID.getError() << std::endl;
        std::cout << "it's me : " << *leftPWM << ";" << *leftSpeedSetpoint << " : " << *rightPWM << ";" << *rightSpeedSetpoint
                  << " : " << *currentDistance << ";" << *translationSetpoint << " : " << leftCurveRatio << ";" << rightCurveRatio
                  << " : " << curveSetpoint << ";" << deltaRadius << std::endl;
    }
    else counter++;

    //std::cout << "PWM time : " << Millis() - time << std::endl;

    direction.setAngle(ARCTAN((double)DIST_MOTOR_DIRECTION / (double)(*curveSetpoint + *deltaRadius)));
    //direction.setAngle(0);
}

void MotionController::stop()
{

    std::cout << "DEBUG : STOP" << std::endl;

    leftMotor.run(0);
    rightMotor.run(0);

    *currentDistance = (odo.getRightValue()+odo.getLeftValue())/2;
    *translationSetpoint = *currentDistance;
    *translationSpeed = 0;
    *leftSpeedSetpoint = 0;
    *rightSpeedSetpoint = 0;
    *leftPWM = 0;
    *rightPWM = 0;

    leftMotor.run(0);
    rightMotor.run(0);

    timespec t, r;
    t.tv_sec=0;
    t.tv_nsec = 1000000;
    nanosleep(&t, &r);

    *currentDistance = (odo.getRightValue()+odo.getLeftValue())/2;
    *translationSetpoint = *currentDistance;
    *translationSpeed = 0;
    *leftSpeedSetpoint = 0;
    *rightSpeedSetpoint = 0;
    *leftPWM = 0;
    *rightPWM = 0;

    leftMotor.run(0);
    rightMotor.run(0);

    leftCurveRatio = 1.0;
    rightCurveRatio = 1.0;
    translationPID.resetErrors();
    leftSpeedPID.resetErrors();
    rightSpeedPID.resetErrors();

    moving = false;

}

void MotionController::manageStop()
{
    static long time = 0;
  //  static uint32_t time2 = 0;
  //  static uint32_t time3 = 0;
  //  static uint32_t timeToEstablish = 0;
  //  static uint32_t timeNotEstablished = 0;
  //  static bool isSpeedEstablished = false;

    if (isPhysicallyStopped() && moving) // Pour un blocage classique
    {

        if (time == 0)
        { //D�but du timer
            time = Millis();
        }
        else
        {
            if ((Millis() - time) >= delayToStop)
            { //Si arr�t� plus de 'delayToStop' ms
                if (ABS(translationPID.getError()) <= toleranceTranslation)
                { //Stop� pour cause de fin de mouvement
                    std::cout << "DEBUG : ARRIVED AT DESTINATION " << translationPID.getError() << " : " << translationSetpoint << std::endl;
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

const char * MotionController::getTunings(void)
{
    return (
            std::string("\nLEFT SPEED : ")+std::to_string(leftSpeedPID.getKp())+std::string(" ")+std::to_string(leftSpeedPID.getKi())+std::string(" ")+std::to_string(leftSpeedPID.getKd())+std::string("\r\n")+
            std::string("RIGHT SPEED : ")+std::to_string(rightSpeedPID.getKp())+std::string(" ")+std::to_string(rightSpeedPID.getKi())+std::string(" ")+std::to_string(rightSpeedPID.getKd())+std::string("\r\n")+
            std::string("TRANSLATION : ")+std::to_string(translationPID.getKp())+std::string(" ")+std::to_string(translationPID.getKi())+std::string(" ")+std::to_string(translationPID.getKd())+std::string("\r\n")+
            std::string("CURVE : ")+std::to_string(curvePID.getKp())+std::string(" ")+std::to_string(curvePID.getKi())+std::string(" ")+std::to_string(curvePID.getKd())+std::string("\r\n\n")
          ).c_str();
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
    *translationSetpoint += (long) ((double)mmDistance / (double)MM_PER_TICK);
    std::cout << "it's me order: " << *translationSetpoint << std::endl;
}

void MotionController::testSpeed(int speed)
{
    *translationSpeed = speed;

    timespec t, r;
    t.tv_sec= 2;
    t.tv_nsec = 0;
    nanosleep(&t, &r);

    stop();
}

void MotionController::orderAngle(float angle)
{
    direction.setAngle(angle);
}

Odometry* MotionController::getOdometry(void)
{
    return &odo;
}

long MotionController::getCurveRadius()
{
    return *currentRadius;
}
