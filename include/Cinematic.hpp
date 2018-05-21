//
// Created by discord on 06/03/17.
//

#ifndef MOTORDAEMON_CINEMATIC_HPP
#define MOTORDAEMON_CINEMATIC_HPP

class Cinematic {
public:

    Cinematic(double rel, double cur, bool way) : relativeDistance(rel), curvePoint(cur), way(way){}

    double relativeDistance;
    double curvePoint;
    bool way;
};

#endif //MOTORDAEMON_CINEMATIC_HPP
