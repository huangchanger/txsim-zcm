#pragma once
#include <vector>
#include <string>
#include <list>
#include <string>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

// #include "rapidjson/document.h"
// #include "rapidjson/filereadstream.h"
// #include "rapidjson/filewritestream.h"

using std::string;
using std::vector;

namespace txsim
{

    namespace simutils
    {
        /*----------------units transform----------------*/

        inline double rad2deg(double rad)
        {
            return rad / M_PI * 180.0;
        }

        inline double deg2rad(double deg)
        {
            return deg * M_PI / 180.0;
        }

        inline double kph2mps(double kph)
        {
            return kph / 3.6;
        }

        inline double mps2kph(double mps)
        {
            return mps * 3.6;
        }

        /*-----------------basic math--------------------*/

        inline double norm2(double x, double y, double z = 0)
        {
            return sqrt(x * x + y * y + z * z);
        }

        /*--------------string operation-----------------*/

        inline bool StartWith(const std::string &s1, const std::string &s2)
        {
            return s2.size() <= s1.size() && s1.compare(0, s2.size(), s2) == 0;
        }

        /*---------------file stream---------------------*/

        // inline rapidjson::Document ReadJsonFile(const string &filepath)
        // {
        //     rapidjson::Document doc;
        //     FILE *fp = fopen(filepath.c_str(), "r");
        //     if (fp == 0)
        //     {
        //         printf("[Error] cannot open %s.\n", filepath.c_str());
        //         return doc;
        //     }
        //     char read_buffer[2048];
        //     rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));
        //     doc.ParseStream(is);
        //     fclose(fp);
        //     return doc;
        // };

        /*---------------vtd specified-------------------*/

        /*---------------tiev-plus specified-------------*/

    } // namespace simutils
} // namespace tievsim