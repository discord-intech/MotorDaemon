//
// Created by discord on 26/09/16.
//

#include "MotionController.hpp"

MotionController::MotionController() : leftMotor(Side::LEFT), rightMotor(Side::RIGHT),
rightSpeedPID(&currentRightSpeed, &rightPWM, &rightSpeedSetpoint),
leftSpeedPID(&currentLeftSpeed, &leftPWM, &leftSpeedSetpoint),
translationPID(&currentDistance, &translationSpeed, &translationSetpoint),
averageLeftSpeed(), averageRightSpeed(), odo(67,68,44,26) //TODO PINS
{
    translationSetpoint = 0;
    leftSpeedSetpoint = 0;
    rightSpeedSetpoint = 0;

    leftSpeedPID.setOutputLimits(-255,255);
    rightSpeedPID.setOutputLimits(-255,255);

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

    translationPID.setTunings(13, 0, 0);
    leftSpeedPID.setTunings(0.01, 0.000025, 0.0001); // ki 0.00001
    rightSpeedPID.setTunings(0.01, 0.000025, 0.0001);

    distanceTest = 200;
}

void MotionController::init()
{
    leftMotor.initPWM();
    rightMotor.initPWM();

    //TODO init thread asserv
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


    /*
     * Comptage des ticks de la roue droite
     * Cette codeuse est connect�e � un timer 16bit
     * on subit donc un overflow/underflow de la valeur des ticks tous les 7 m�tres environ
     * ceci est corrig� de mani�re � pouvoir parcourir des distances grandes sans devenir fou en chemin (^_^)
     */
    static long lastRawRightTicks = 0;	//On garde en m�moire le nombre de ticks obtenu au pr�c�dent appel
    static int rightOverflow = 0;			//On garde en m�moire le nombre de fois que l'on a overflow (n�gatif pour les underflow)

    long rawRightTicks = odo.getRightValue();	//Nombre de ticks avant tout traitement

    if (lastRawRightTicks - rawRightTicks > 32768)		//D�tection d'un overflow
        rightOverflow++;
    else if(lastRawRightTicks - rawRightTicks < -32768)	//D�tection d'un underflow
        rightOverflow--;

    lastRawRightTicks = rawRightTicks;

    long rightTicks = rawRightTicks + rightOverflow*65535;	//On calcule le nombre r�el de ticks

    /*
     * Comptage des ticks de la roue gauche
     * ici on est sur un timer 32bit, pas de probl�me d'overflow sauf si on tente de parcourir plus de 446km...
     */
    long leftTicks = odo.getLeftValue();


    currentLeftSpeed = (leftTicks - previousLeftTicks)*2000; // (nb-de-tick-passés)*(freq_asserv) (ticks/sec)
    currentRightSpeed = (rightTicks - previousRightTicks)*2000;

    previousLeftTicks = leftTicks;
    previousRightTicks = rightTicks;

    averageLeftSpeed.add(currentLeftSpeed);
    averageRightSpeed.add(currentRightSpeed);

    currentLeftSpeed = averageLeftSpeed.value(); // On utilise pour l'asserv la valeur moyenne des dernieres current Speed
    currentRightSpeed = averageRightSpeed.value(); // sinon le robot il fait nawak.


    currentDistance = (leftTicks + rightTicks) / 2;
    currentAngle = ((rightTicks - currentDistance)*RAYON_COD_GAUCHE/RAYON_COD_DROITE - (leftTicks - currentDistance)) / 2;


    translationPID.compute();	// Actualise la valeur de 'translationSpeed'


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
