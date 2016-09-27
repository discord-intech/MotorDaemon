#include <iostream>
#include "Odometry.hpp"

int main() {

    Odometry odo(7,8,9,10);

    while(true)
    {
        std::cout << odo.getLeftValue() << " ; " << odo.getRightValue();

        g_usleep(1000*500);
    }

    return 0;
}