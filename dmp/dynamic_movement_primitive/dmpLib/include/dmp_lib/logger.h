/*************************************************************************
	> File Name: logger.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 17 Oct 2016 09:22:34 PM PDT
 ************************************************************************/

#ifndef _LOGGER_H
#define _LOGGER_H

// system include
#include <string>
#include <stddef.h>
#include <stdio.h>

namespace dmp_lib
{
    class Logger
    {
        public:
            enum Level
            {
                DEBUG=1, INFO, WARN, ERROR, FATAL
            };
        
            /*!
             * @param msg
             * @param ;eve;
             * @return
             */
            static void logPrintf(const char *msg, Logger::Level level = Logger::INFO, ...);

            /*!
             * @param condition
             * @param msg
             * @param Level
             * @return
             */
            static void logPrintf(bool condition, const char *msg, Logger::Level level = Logger::INFO, ...);

            /*!
             *  @param Level
             */
            static void setLogLevel(Logger::Level level)
            {
                level_ = level;
            }

            /*!
             * @param Level
             * @return 
             */
            static std::string getLogLevel(Logger::Level level);

        private:
            /*! Constructor
             */
            Logger() {};

            /*! Destructor
             */
            virtual ~Logger() {};

            /*!
             * @param level 
             * @return
             */
            static bool toBeLogged(Logger::Level level)
            {
                return (level >= level_);
            }

            static Level level_;
    };

    // Inline function follow
    inline std::string Logger::getLogLevel(Logger::Level level)
    {
        std::string level_string;
        switch (level)
        {
            case DEBUG:
                level_string.assign("DEBUG");
                break;
            case INFO:
                level_string.assign("INFO");
                break;
            case WARN:
                level_string.assign("WARN");
                break;
            case ERROR:
                level_string.assign("ERROR");
                break;
            case FATAL:
                level_string.assign("FATAL");
                break;
        }
        return level_string;
    }
}
#endif // _LOGGER_H
