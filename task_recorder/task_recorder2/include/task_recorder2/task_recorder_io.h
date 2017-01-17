/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks   ...

 \file    task_recorder_io.h

 \author  Peter Pastor, Mrinal Kalakrishnan
 \date    Jul 14, 2010

 *********************************************************************/

#ifndef TASK_RECORDER_IO_H_
#define TASK_RECORDER_IO_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <boost/filesystem.hpp>
#include <boost/scoped_ptr.hpp>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <task_recorder2_msgs/DataSample.h>
#include <task_recorder2_msgs/DataSampleLabel.h>

#include <task_recorder2_msgs/Description.h>
#include <task_recorder2_msgs/AccumulatedTrialStatistics.h>
#include <task_recorder2_utilities/task_recorder_utilities.h>
#include <task_recorder2_utilities/task_description_utilities.h>

#include <dmp_lib/trajectory.h>

// local includes

namespace task_recorder2
{

// default template parameters
template<class MessageType = task_recorder2_msgs::DataSample> class TaskRecorderIO;

template<class MessageType>
  class TaskRecorderIO
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    TaskRecorderIO(ros::NodeHandle node_handle) :
      node_handle_(node_handle), write_out_raw_data_(false), write_out_clmc_data_(false), write_out_resampled_data_(false), write_out_statistics_(false),
          initialized_(false) {};
    /*! Destructor
     */
    virtual ~TaskRecorderIO() {};

    /*!
     * @param topic_name
     * @return True on success, otherwise False
     */
    bool initialize(const std::string& topic_name);

    /*!
     * @param description
     * @param directory_name
     */
    void setDescription(const task_recorder2_msgs::Description& description,
                        const std::string directory_name = std::string(""));
    void setResampledDescription(const task_recorder2_msgs::Description& description,
                        const std::string directory_name = std::string("resampled"));
    task_recorder2_msgs::Description getDescription() const;

    /*!
     * @param directory_name
     * @param increment_trial_counter
     * @return True on success, otherwise False
     */
    bool writeRecordedData(const std::string directory_name, bool increment_trial_counter);
    bool writeResampledData();

    /*!
     * @param directory_name
     * @return True on success, otherwise False
     */
    bool writeRecordedDataToCLMCFile(const std::string directory_name = std::string(""));

    /*!
     * @return True on success, otherwise False
     */
    bool writeRecordedDataSamples();

    /*!
     * @param raw_directory_name
     * @return True on success, otherwise False
     */
    bool writeRawData(const std::string raw_directory_name);
    bool writeRawData();

    /*!
     * @return True on success, otherwise False
     */
    bool writeStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics);

    /*!
     */
    ros::NodeHandle node_handle_;
    std::string topic_name_;

    /*!
     */
    std::vector<MessageType> messages_;
    bool write_out_raw_data_;
    bool write_out_clmc_data_;
    bool write_out_resampled_data_;
    bool write_out_statistics_;

  private:

    /*!
     */
    int trial_;
    bool initialized_;

    /*!
     */
    task_recorder2_msgs::Description description_;
    std::string data_directory_name_;

    /*!
     */
    boost::filesystem::path absolute_data_directory_path_;
    bool create_directories_;

  };

template<class MessageType>
  bool TaskRecorderIO<MessageType>::initialize(const std::string& topic_name)
  {
    topic_name_ = topic_name;
    ROS_INFO("Initializing task recorder for topic named >%s<.", topic_name_.c_str());

    node_handle_.param("create_directories", create_directories_, true);
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_resampled_data", write_out_resampled_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_raw_data", write_out_raw_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_clmc_data", write_out_clmc_data_));
    ROS_VERIFY(usc_utilities::read(node_handle_, "write_out_statistics", write_out_statistics_));

    std::string recorder_package_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_package_name", recorder_package_name));
    std::string recorder_data_directory_name;
    ROS_VERIFY(usc_utilities::read(node_handle_, "recorder_data_directory_name", recorder_data_directory_name));
    data_directory_name_ = task_recorder2_utilities::getDirectoryPath(recorder_package_name, recorder_data_directory_name);
    ROS_VERIFY(task_recorder2_utilities::checkAndCreateDirectories(data_directory_name_));
    ROS_DEBUG("Setting TaskRecorderIO data directory name to >%s<.", data_directory_name_.c_str());

    return (initialized_ = true);
  }

template<class MessageType>
  void TaskRecorderIO<MessageType>::setDescription(const task_recorder2_msgs::Description& description,
                                                   const std::string directory_name)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    description_ = description;

    if(create_directories_)
    {
      // check whether directory exists, if not, create it
      absolute_data_directory_path_ = boost::filesystem::path(data_directory_name_ + task_recorder2_utilities::getFileName(description_));
      boost::filesystem::path path = absolute_data_directory_path_;
      if(!directory_name.empty())
      {
        path = boost::filesystem::path(absolute_data_directory_path_.string() + std::string("/") + directory_name);
      }
      ROS_VERIFY(task_recorder2_utilities::checkForDirectory(path));
      ROS_VERIFY(task_recorder2_utilities::getTrialId(path, trial_, topic_name_));
      ROS_VERIFY(task_recorder2_utilities::checkForCompleteness(path, trial_, topic_name_));
      description_.trial = trial_;
    }
    else
    {
      boost::filesystem::path path = boost::filesystem::path(data_directory_name_);
      ROS_VERIFY(task_recorder2_utilities::checkForDirectory(path));
      absolute_data_directory_path_ = boost::filesystem::path(data_directory_name_ + task_recorder2_utilities::getBagFileName(description_));
    }
  }

template<class MessageType>
  void TaskRecorderIO<MessageType>::setResampledDescription(const task_recorder2_msgs::Description& description,
                                                            const std::string directory_name)
  {
    setDescription(description, directory_name);
  }

template<class MessageType>
  task_recorder2_msgs::Description TaskRecorderIO<MessageType>::getDescription() const
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    return description_;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedData(const std::string directory_name,
                                                      bool increment_trial_counter)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");

    if(create_directories_)
    {
      std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path_);
      boost::filesystem::path path = absolute_data_directory_path_;
      if (!directory_name.empty())
      {
        file_name.append(directory_name);
        path = boost::filesystem::path(absolute_data_directory_path_.string() + std::string("/") + directory_name);
        ROS_VERIFY(task_recorder2_utilities::checkForDirectory(file_name));
        usc_utilities::appendTrailingSlash(file_name);
      }
      file_name.append(task_recorder2_utilities::getDataFileName(topic_name_, trial_));
      ROS_VERIFY(usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messages_, topic_name_, file_name, false));
      if (increment_trial_counter)
      {
        ROS_VERIFY(task_recorder2_utilities::incrementTrialCounterFile(path, topic_name_));
        ROS_VERIFY(task_recorder2_utilities::getTrialId(path, trial_, topic_name_));
        ROS_VERIFY(task_recorder2_utilities::checkForCompleteness(path, trial_, topic_name_));
      }
    }
    else
    {
      std::string file_name = absolute_data_directory_path_.filename().string();
      ROS_VERIFY(usc_utilities::FileIO<MessageType>::writeToBagFileWithTimeStamps(messages_, topic_name_, file_name, false));
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedDataToCLMCFile(const std::string directory_name)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    ROS_ASSERT_MSG(!messages_.empty(), "Messages are empty. Cannot write anything to CLMC file.");
    if(create_directories_)
    {
      std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path_);
      boost::filesystem::path path = absolute_data_directory_path_;
      if (!directory_name.empty())
      {
        file_name.append(directory_name);
        path = boost::filesystem::path(absolute_data_directory_path_.string() + std::string("/") + directory_name);
        ROS_VERIFY(task_recorder2_utilities::checkForDirectory(file_name));
        usc_utilities::appendTrailingSlash(file_name);
      }
      std::string clmc_file_name;
      ROS_VERIFY(task_recorder2_utilities::setCLMCFileName(clmc_file_name, trial_ - 1));
      file_name.append(clmc_file_name);

      const int trajectory_length = (int)messages_.size();
      double trajectory_duration = (messages_[trajectory_length-1].header.stamp - messages_[0].header.stamp).toSec();
      if(trajectory_length == 1) // TODO: think about this again...
      {
        trajectory_duration = 1.0;
      }
      ROS_ASSERT_MSG(trajectory_duration > 0.0, "Trajectory duration >%f< of trajectory named >%s< must be possitive.", trajectory_duration, file_name.c_str());
      const double sampling_frequency = (double)trajectory_length / trajectory_duration;
      boost::scoped_ptr<dmp_lib::Trajectory> trajectory(new dmp_lib::Trajectory());
      ROS_VERIFY(trajectory->initialize(messages_[0].names, sampling_frequency, true, trajectory_length));
      for(int i=0; i<trajectory_length; ++i)
      {
        ROS_VERIFY(trajectory->add(messages_[i].data, true));
      }
      ROS_VERIFY(trajectory->writeToCLMCFile(file_name, true));
    }
    else
    {
      ROS_ERROR("Cannot write CLMC file when \"create_directories\" is disabled.");
      return false;
    }
    return true;
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeResampledData()
  {
    return writeRecordedData(std::string("resampled"), true);
  }
template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRecordedDataSamples()
  {
    return writeRecordedData(std::string(""), true);
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRawData(const std::string raw_directory_name)
  {
    return writeRecordedData(raw_directory_name, false);
  }
template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeRawData()
  {
    return writeRecordedData(std::string("raw"), false);
  }

template<class MessageType>
  bool TaskRecorderIO<MessageType>::writeStatistics(std::vector<std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics)
  {
    ROS_ASSERT_MSG(initialized_, "Task recorder IO module is not initialized.");
    std::string file_name = task_recorder2_utilities::getPathNameIncludingTrailingSlash(absolute_data_directory_path_) + task_recorder2_utilities::getStatFileName(topic_name_, trial_);
    try
    {
      rosbag::Bag bag;
      bag.open(file_name, rosbag::bagmode::Write);
      for (int i = 0; i < static_cast<int> (vector_of_accumulated_trial_statistics.size()); ++i)
      {
        std::vector<task_recorder2_msgs::AccumulatedTrialStatistics> accumulated_trial_statistics = vector_of_accumulated_trial_statistics[i];
        for (int j = 0; j < static_cast<int> (accumulated_trial_statistics.size()); ++j)
        {
          // accumulated_trial_statistics[j].id = getId(description_);
          bag.write(topic_name_, messages_[j].header.stamp, accumulated_trial_statistics[j]);
        }
      }
      bag.close();
    }
    catch (rosbag::BagIOException& ex)
    {
      ROS_ERROR("Problem when writing to bag file named >%s< : %s", file_name.c_str(), ex.what());
      return false;
    }
    return true;
  }

}

#endif /* TASK_RECORDER_IO_H_ */
