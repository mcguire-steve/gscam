
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gscam/gspipeline_nodelet.h>

PLUGINLIB_DECLARE_CLASS(gscam, GSPipelineNodelet, gscam::GSPipelineNodelet, nodelet::Nodelet) 

namespace gscam {
  GSPipelineNodelet::GSPipelineNodelet() :
    nodelet::Nodelet(),
    gspipeline_driver_(NULL),
    stream_thread_(NULL)
  {
  }

  GSPipelineNodelet::~GSPipelineNodelet() 
  {
    stream_thread_->join();
  }

  void GSPipelineNodelet::onInit()
  {
    gspipeline_driver_.reset(new gscam::GSPipeline(this->getNodeHandle(), this->getPrivateNodeHandle()));
    stream_thread_.reset(new boost::thread(boost::bind(&GSPipeline::run, gspipeline_driver_.get())));
  }
}
