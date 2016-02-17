#ifndef __GSCAM_GSPIPELINE_NODELET_H
#define __GSCAM_GSPIPELINE_NODELET_H

#include <nodelet/nodelet.h>

#include <gscam/gspipeline.h>

#include <boost/thread.hpp>

namespace gscam {
  class GSPipelineNodelet : public nodelet::Nodelet
  {
  public:
    GSPipelineNodelet();
    ~GSPipelineNodelet();

    virtual void onInit();

  private:
    boost::scoped_ptr<GSPipeline> gspipeline_driver_;
    boost::scoped_ptr<boost::thread> stream_thread_;
  };
}

#endif // infdef __GSCAM_GSCAM_NODELET_H
