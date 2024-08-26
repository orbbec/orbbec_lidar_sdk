#pragma once
#include "frame.hpp"
namespace ob_lidar_driver {

class Filter {
   public:
    Filter() = default;
    virtual ~Filter() = default;
    virtual void process(std::shared_ptr<Frame> frame) = 0;
};

class SmoothFilter : public Filter {
   public:
    SmoothFilter(float radius);

    ~SmoothFilter() override = default;

    void process(std::shared_ptr<Frame> frame) override;

   private:
    float radius_;
};

}  // namespace ob_lidar_driver