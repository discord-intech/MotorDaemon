#include <iostream>
#include "Odometry.hpp"

int main() {

    Odometry odo(67,68,44,26);

    std::cout << "Go !" << std::endl;

    while(true)
    {
        std::cout << odo.getLeftValue() << " ; " << odo.getRightValue() << std::endl;

        usleep(1000*500);
    }

    return 0;
}