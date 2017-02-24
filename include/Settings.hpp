/**
 * Settings handler for MotorDaemon
 * @author discord
 */

#ifndef MOTORDAEMON_SETTINGS_HPP
#define MOTORDAEMON_SETTINGS_HPP

#include <iostream>
#include <unordered_map>

class Settings {
private:

    std::string path;

    std::unordered_map<std::string, std::string> settings;

    void parse(void);

public:

    Settings(std::string);

    std::string get(std::string);

};

#endif //MOTORDAEMON_SETTINGS_HPP
