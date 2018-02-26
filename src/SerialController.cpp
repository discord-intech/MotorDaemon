//
// Created by discord on 2/20/18.
//



#include "../include/SerialController.hpp"



bool SerialController::on = true;
int SerialController::fileDesc;
struct cpu_com_status* SerialController::currentStatus;
std::queue<Result *> SerialController::resultQueue;

SerialController::SerialController(char* port)
{
    this->fileDesc = open (port, O_RDWR | O_NOCTTY | O_SYNC);
    if (this->fileDesc < 0)
    {
        printf("error %d opening %s: %s", errno, port, strerror (errno));
        return;
    }

    set_interface_attribs (this->fileDesc, B115200, 0);		// set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (this->fileDesc, 1);
}

void SerialController::init()
{
    currentStatus = static_cast<cpu_com_status *>(malloc(sizeof(struct cpu_com_status)));
    currentStatus->stop = true;
    main = std::thread(&SerialController::mainWorker);
    main.detach();
    reader = std::thread(&SerialController::readWorker);
    reader.detach();
    resultQueue = std::queue<Result *>();

    system((std::string("echo ")+std::to_string(57)+std::string(" > /sys/class/gpio/export")).c_str());

    system((std::string("echo out > /sys/class/gpio/gpio")+std::to_string(57)+std::string("/direction")).c_str());
    system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(57)+std::string("/value")).c_str());

}

int SerialController::Read(char *b, int size)
{
    return read(fileDesc, b, size);
}

int SerialController::Read_until(char *b, int maxSize, char finalChar)
{
    for(int i=0 ; i<maxSize ; i++)
    {
        char c;
        read(fileDesc, &c, 1);

        if (c == finalChar)
        {
            b[i] = '\0';
            return i;
        }

        b[i] = c;
    }
}

int SerialController::Write(const char *b, unsigned int size)
{
    return write(fileDesc, b, size);
}

void SerialController::mainWorker()
{

}

void SerialController::readWorker()
{
    on = true;
    char buffer[1024];
    while(on)
    {
        char code = 0;

        if(!Read(&code, 1))
        {
            continue;
        }

        if(code == STATUS_CODE)
        {
            delete currentStatus;
            currentStatus = static_cast<struct cpu_com_status*>(malloc(sizeof(struct cpu_com_status)));
            Read_until(reinterpret_cast<char *>(&buffer), 1024, 13);
            nlohmann::json js = nlohmann::json::parse(buffer);

            currentStatus->x = js["x"].get<double>();
            currentStatus->y = js["y"].get<double>();
            currentStatus->angle = js["angle"].get<double>();
            currentStatus->stop = js["stop"].get<bool>();
            currentStatus->curveRadius = js["curveRadius"].get<double>();
            currentStatus->speedL = js["speedL"].get<double>();
            currentStatus->speedR = js["speedR"].get<double>();
            currentStatus->pwmL = js["pwmL"].get<int>();
            currentStatus->pwmR = js["pwmR"].get<int>();
            currentStatus->stopPhy = js["stopPhy"].get<int>() != 0;
            currentStatus->stopSoft = js["stopSoft"].get<int>() != 0;

            std::cout << "GOT STATUS" << std::endl;
        }
        else if(code == RESULT_CODE)
        {
            Result * res = static_cast<Result *>(malloc(sizeof(Result
                                                        )));
            Read_until(reinterpret_cast<char *>(&buffer), 1024, 13);

            nlohmann::json js = nlohmann::json::parse(buffer);

            res->resultCode = js["code"].get<int>();
            memcpy(res->content, js["content"].get<std::string>().c_str(), js["content"].get<std::string>().length());

            resultQueue.push(res);
        }
        else continue;
    }
}

struct cpu_com_status SerialController::getStatus()
{
    struct cpu_com_status status{};

    memcpy(&status, (const void *) currentStatus, sizeof(struct cpu_com_status));

    return status;
}


void SerialController::order(std::string order)
{
    Write((order+"\r").c_str(), static_cast<unsigned int>(order.length() + 1));
}

Result * SerialController::waitForResult()
{
    while (resultQueue.empty())
    {
        timespec t, r;
        t.tv_sec=0;
        t.tv_nsec = 1000000;
        nanosleep(&t, &r);
    }
    Result * val = resultQueue.front();
    resultQueue.pop();
    return val;
}

////////////////////////////////////
///         OVERRIDES            ///
////////////////////////////////////

void SerialController::stop(void)
{
    order("stop");
    waitForResult();
}

void SerialController::orderTranslation(long i) {
    order("d "+std::to_string(i));
    waitForResult();
}

void SerialController::orderAngle(float d) {
    //Not yet implemented in protocol
}

void SerialController::setSpeedTranslation(int i) {
    order("setspeed "+std::to_string(i));
    waitForResult();
}

void SerialController::orderCurveRadius(long i) {
    order("cr "+std::to_string(i));
    waitForResult();
}

void SerialController::setTranslationTunings(float d, float d1, float d2) {
    order("setConsts -1 -1 -1 -1 -1 -1 "+std::to_string(d)+" "+std::to_string(d1)+" "+std::to_string(d2)
          +" -1 -1 -1");
    waitForResult();
}

void SerialController::setCurveTunings(float d, float d1, float d2) {
    order("setConsts -1 -1 -1 -1 -1 -1 -1 -1 -1 "
          +std::to_string(d)+" "+std::to_string(d1)+" "+std::to_string(d2));
    waitForResult();
}

void SerialController::setLeftSpeedTunings(float d, float d1, float d2) {
    order("setConsts "+std::to_string(d)+" "+std::to_string(d1)+" "+std::to_string(d2)
          +" -1 -1 -1 -1 -1 -1 -1 -1 -1");
    waitForResult();
}

void SerialController::setRightSpeedTunings(float d, float d1, float d2) {
    order("setConsts -1 -1 -1 "+std::to_string(d)+" "+std::to_string(d1)+" "+std::to_string(d2)
          +" -1 -1 -1 -1 -1 -1");
    waitForResult();
}

void SerialController::setPosition(double xn, double yn) {
    order("setpos "+std::to_string(xn)+" "+std::to_string(yn));
    waitForResult();
}

void SerialController::setAngle(double o) {
    order("setangle "+std::to_string(o));
    waitForResult();
}

const char *SerialController::getTunings(void) {
    return "Not yet implemented in protocol";
}

void SerialController::testPosition(void) {
    order("testpos "+std::to_string(100));
    waitForResult();
}

void SerialController::testSpeed(int i) {
    order("testspeed "+std::to_string(i));
    waitForResult();
}

void SerialController::setTrajectory(std::vector<Cinematic> &vector, long i) {
    std::string points = "";
    for(Cinematic &c : vector)
    {
        points += ";"+std::to_string(c.relativeDistance)+":"+std::to_string(c.curvePoint);
    }

    order("traj "+std::to_string(i)+":"+std::to_string(vector.size())+points);
    waitForResult();
}

const char *SerialController::isMoving(void) {
    return !currentStatus->stop ? "Yes" : "No";
}

bool SerialController::isPhysicallyStopped(void) {
    return currentStatus->stop;
}

long SerialController::getTranslationSetPoint(void) {
    //Not yet implemented in protocol
    return 0;
}

void SerialController::go(void) {
    //Not yet implemented in protocol
}

void SerialController::goR(void) {
    //Not yet implemented in protocol
}

void SerialController::setControlled(bool b) {
    if(b)
        system((std::string("echo 1 > /sys/class/gpio/gpio")+std::to_string(57)+std::string("/value")).c_str());
    else
        system((std::string("echo 0 > /sys/class/gpio/gpio")+std::to_string(57)+std::string("/value")).c_str());
}

void SerialController::sweep(bool way) {
    //Not yet implemented in protocol
}

void SerialController::stopSweep(void) {
    //Not yet implemented in protocol
}

long SerialController::getCurveRadius(void) {
    return currentStatus->curveRadius;
}

double SerialController::getX(void) {
    return currentStatus->x;
}

double SerialController::getY(void) {
    return currentStatus->y;
}

long SerialController::getSpeed(void) {
    return static_cast<long>((currentStatus->speedL + currentStatus->speedR) / 2.0);
}

long SerialController::getSpeedL(void) {
    return static_cast<long>(currentStatus->speedL);
}

long SerialController::getSpeedR(void) {
    return static_cast<long>(currentStatus->speedR);
}

long SerialController::getCSpeedL(void) {
    //Not yet implemented in protocol
    return 0;
}

long SerialController::getCSpeedR(void) {
    //Not yet implemented in protocol
    return 0;
}

double SerialController::getAngle(void) {
    return currentStatus->angle;
}

void SerialController::loadPos() {
    std::fstream outPos;
    outPos.open("/var/cache/MDPOS", std::ios::in);

    outPos.get();

    std::string pos;
    outPos >> pos;
    std::vector<std::string> comps = splitl(pos, ';');

    this->setPosition(std::stod(comps[0]), std::stod(comps[1]));
    this->setAngle(std::stod(comps[2]));

    outPos.close();
}

void SerialController::printTranslationError(void) {
    //Not yet implemented in protocol
}

const char *SerialController::controlledStatus() {
    if(currentStatus->stopPhy)
    {
        if(currentStatus->stopSoft) return "Phy:Yes;Soft:Yes";
        else return "Phy:Yes;Soft:No";
    }
    else
    {
        if(currentStatus->stopSoft) return "Phy:No;Soft:Yes";
        else return "Phy:No;Soft:No";
    }
}


