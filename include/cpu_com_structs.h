//
// Created by discord on 2/19/18.
//

#ifndef MOTORDAEMON_CPU_COM_STRUCTS_H
#define MOTORDAEMON_CPU_COM_STRUCTS_H

#define CPU_RESULT_BUFFER_SIZE 200
#define STATUS_CODE_1 17
#define STATUS_CODE_2 18
#define RESULT_CODE_1 19
#define RESULT_CODE_2 20

struct cpu_com_result
{
    int resultCode;
    char content[CPU_RESULT_BUFFER_SIZE];
};

struct cpu_com_status
{
    double x, y;
    double angle;
    bool stop;
    long curveRadius;
    double speedL, speedR;
    char pwmL, pwmR;
    bool stopPhy, stopSoft, ampOverload;
};

#endif //MOTORDAEMON_CPU_COM_STRUCTS_H
