//
// Created by discord on 24/02/17.
//
#include "Settings.hpp"

#define NO_SETTINGS_FAIL "cannot_find_setting"

/**
 * Constructor
 * @param p path to config file
 */
Settings::Settings(std::string p)
{
    this->path = p;
    parse();
}

void Settings::parse(void)
{
    FILE* file = fopen(path.c_str(), "r");

    if(file == NULL)
    {
        std::cerr << "Can't open settings file " << path << " !" << std::endl;
        return;
    }

    std::string content = "";
    int currentChar;

    int ignore = 0;

    while((currentChar = fgetc(file)) != EOF)
    {

        if(ignore == 0)
        {
            if (currentChar == '#')
            {
                ignore = 1;
                continue;
            }

            else if (currentChar == '/')
            {
                int nextChar = fgetc(file);

                if(nextChar == '*')
                {
                    ignore = 2;
                    continue;
                }

                fseek(file, -1, SEEK_CUR);
            }

            else if (currentChar == '\\') // Ignoring special chars with backslash
            {
                int nextChar = fgetc(file);

                if(nextChar == '/' || nextChar == '#')
                {
                    content += (char)nextChar;
                    continue;
                }

                fseek(file, -1, SEEK_CUR);
            }

            content += (char)currentChar;
        }
        else
        {
            if (ignore == 1 && currentChar == '\n')
            {
                ignore = 0;
            }
            else if(ignore == 2 && currentChar == '*')
            {
                int nextChar = fgetc(file);

                if(nextChar == '/')
                {
                    ignore = 0;
                    continue;
                }

                fseek(file, -1, SEEK_CUR);
            }
        }

    }

    fclose(file);

    std::string name, value;
    int mode = 0;

    name.clear();
    value.clear();

    for(char c : content)
    {

        if(name.empty())
        {
            if(c == ' ') continue;
        }

        if(c == '=')
        {
            mode = 1;
            continue;
        }
        else if(c == '\r') continue;
        else if(c == '\n')
        {
            mode = 0;
            if(!name.empty()) settings.emplace(name, value);
            name.clear();
            value.clear();
            continue;
        }

        if(mode)value += c;
        else name+=c;
    }

    if(!name.empty()) settings.emplace(name, value);

}

std::string Settings::get(std::string name)
{
    if(settings.count(name) == 0)
    {
        return NO_SETTINGS_FAIL;
    }

    return settings[name];
}