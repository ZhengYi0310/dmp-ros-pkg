/*************************************************************************
	> File Name: status.h
	> Author: Yi Zheng 
	> Mail: hczhengcq@gmail.com
	> Created Time: Mon 17 Oct 2016 11:10:03 PM PDT
 ************************************************************************/

#ifndef DMP_STATUS_H
#define DMP_STATUS_H

// system includes
// local includes

namespace dmp_lib
{
    class Status 
    {
        public:

            /*! Constructor
             */
            Status() : initialized_(false) {};
            
            /*! Destructor
             */
            virtual ~Status() {};

            /*!
             * @return True if initialized, otherwise False 
             */
            bool isInitialized() const;

        protected:
            /*!
             */
            bool initialized_;
    };

    // Inline functions follow
    inline bool Status::isInitialized() const
    {
        return initialized_;
    }
}
#endif
