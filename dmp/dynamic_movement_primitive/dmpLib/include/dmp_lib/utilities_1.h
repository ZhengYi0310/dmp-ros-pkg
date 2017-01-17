/*************************************************************************
	> File Name: utilities.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Wed 26 Oct 2016 12:19:35 PM PDT
 ************************************************************************/

#ifndef _UTILITIES_H
#define _UTILITIES_H

// system includes 
#include <vector>
#include <boost/shared_ptr.hpp>

// local includes 
#include "dmp_lib/logger.h"

namespace dmp_lib
{
    template<class T>
    class Utilities 
    {
        public:
            
            /*!
             * @param dest 
             * @param src 
             */
            static bool assign(boost::shared_ptr<T>& dest, const boost::shared_ptr<T> src);

            /*!
             * @param dest 
             * @param src
             */
            static bool assign(std::vector<boost::shared_ptr<T> >& dest, const std::vector<boost::shared_ptr<T> > src);

        private:
            Utilities() {};
            virtual ~Utilities() {};
    };

    template<class T>
    bool Utilities<T>::assign(boost::shared_ptr<T>& dest, const boost::shared_ptr<T> src)
    {
        if (!src.get())
        {
            Logger::logPrintf("Pointer not set. Cannot assign it!", Logger::FATAL);
            return false;
        }
        dest.reset(new T());
        *dest = *src;
        return true;
    }

    template<class T>
    bool Utilities<T>::assign(std::vector<boost::shared_ptr<T> >& dest, const std::vector<boost::shared_ptr<T> > src)
    {
        if (src.empty())
        {
            Logger::logPrintf("Cannot assign vector of shared pointer from empty vector.", Logger::FATAL);
            return false;
        }
        dest.clear();
        for (int i = 0; i < (int)src.size(); ++i)
        {
            boost::shared_ptr<T> t;
            if (!Utilities::assign(t, src[i]))
            {
                return false;
            }
            dest.push_back(t);
            //Logger::logPrintf("dest vector size %i.", Logger::DEBUG, dest.size());
        }
        return true;
    }
}
#endif /* _UTILITIES_H */
