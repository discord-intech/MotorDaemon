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

    const std::string path;

    std::unordered_map<std::string, std::string> settings;

    void parse(void);

public:

    Settings(const std::string&);

    std::string get(const std::string&);

    double getDouble(const std::string&);

    int getInt(const std::string&);

    float getFloat(const std::string&);
};

class FailedToParse{};

#endif //MOTORDAEMON_SETTINGS_HPP
