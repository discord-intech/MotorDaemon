//
// Created by discord on 06/03/17.
//

#ifndef MOTORDAEMON_CINEMATIC_HPP
#define MOTORDAEMON_CINEMATIC_HPP

class Cinematic {
public:

    Cinematic(double rel, double cur) : relativeDistance(rel), curvePoint(cur){}

    double relativeDistance;
    double curvePoint;
};

#endif //MOTORDAEMON_CINEMATIC_HPP
