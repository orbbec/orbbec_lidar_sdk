#include "filter.hpp"

#include <frame.hpp>
#include <stdexcept>
namespace ob_lidar::detail {

OutlierRemovalFilterImpl::OutlierRemovalFilterImpl(int level, int scan_speed)
    : level_(level), scan_speed_(scan_speed) {}

OutlierRemovalFilterImpl::~OutlierRemovalFilterImpl() = default;

void OutlierRemovalFilterImpl::setFilterWindowSize(int window_size) {
    if (window_size % 2 == 0) {
        throw std::invalid_argument("Window size must be odd number");
    }
    window_size_ = window_size;
}

void OutlierRemovalFilterImpl::filter(std::vector<uint16_t>& ranges) const {
    uint16_t sum_left_abnormal =
        0;  // Number of abnormal points on the left side of the sliding window
    uint16_t sum_right_abnormal =
        0;  // Number of abnormal points on the right side of the sliding window
    int32_t neighbors = 0;  // Number of neighboring points to be filtered
    int32_t point_num_threshold =
        0;  // Threshold for the number of adjacent points exceeding the limit
    float tan_angle_threshold = 0;  // Threshold for point cloud filtering based
                                    // on the tangent of the angle
    float angle_res_sin = 0;        // Sine of the angle interval
    float angle_res_cos = 0;        // Cosine of the angle interval
    double tan_angle = 0;           // Tangent of the angle between points
    uint16_t abnormal_point_diff =
        25;  // Pre-judgment difference with points on the left and right sides
    uint16_t point_distance_cmp =
        0;  // Temporary value for point cloud distance comparison
    uint16_t cur_distance = 0;  // Current distance value
    uint16_t diff_1, diff_2;    // Differences in distances for comparison
    const size_t count = ranges.size();  // Number of points in one cycle

    std::vector<uint16_t> temp_distance_data;

    if (count < window_size_) {
        return;
    }

    if (level_ == 0) {
        return;
    }

    if ((scan_speed_ != 10) && (scan_speed_ != 15) && (scan_speed_ != 20) &&
        (scan_speed_ != 25) && (scan_speed_ != 30)) {
        return;
    }

    // Resize the temporary distance data vector to match the number of points
    temp_distance_data.resize(count);
    std::copy(ranges.begin(), ranges.end(), temp_distance_data.begin());

    switch (level_) {
        case 1:
            // tan(3°), the trailing or noise angle range is within ±4°,
            // remove 1 adjacent point, at least 3 points on one side meet the
            // criteria
            tan_angle_threshold = 0.052407779283041;
            neighbors = 1;
            point_num_threshold = 3;
            break;
        case 2:
            // tan(3°), the trailing or noise angle range is within ±4°,
            // remove 1 adjacent point, at least 2 points on one side meet the
            // criteria
            tan_angle_threshold = 0.052407779283041;
            neighbors = 1;
            point_num_threshold = 2;
            break;
        case 3:
            // tan(5.5°), the trailing or noise angle range is within ±7°,
            // remove 1 adjacent point, at least 2 points on one side meet the
            // criteria
            tan_angle_threshold = 0.096289048197539;
            neighbors = 1;
            point_num_threshold = 2;
            break;
        case 4:
            // tan(9°), the trailing or noise angle range is within ±9°,
            // remove 1 adjacent point, at least 2 points on one side meet the
            // criteria
            tan_angle_threshold = 0.158384440324536;
            neighbors = 1;
            point_num_threshold = 2;
            break;
        case 5:
            // tan(9°), the trailing or noise angle range is within ±9°,
            // remove 1 adjacent point, at least 1 point on one side meets the
            // criteria
            tan_angle_threshold = 0.158384440324536;
            neighbors = 1;
            point_num_threshold = 1;
            break;
        default:
            tan_angle_threshold = 0;
            break;
    }

    switch (scan_speed_) {
        case 10:
            angle_res_sin = -0.002073449665673;
            angle_res_cos = 0.999997850400932;
            abnormal_point_diff = 15;
            break;
        case 15:
            angle_res_sin = -0.003110171712830;
            angle_res_cos = 0.999995163404262;
            abnormal_point_diff = 30;
            break;
        case 20:
            angle_res_sin = -0.004146890417175;
            angle_res_cos = 0.999991401612968;
            abnormal_point_diff = 25;
            break;
        case 25:
            angle_res_sin = -0.005183604664443;
            angle_res_cos = 0.999986565031092;
            abnormal_point_diff = 35;
            break;
        case 30:
            angle_res_sin = -0.006220313340373;
            angle_res_cos = 0.999980653663833;
            abnormal_point_diff = 40;
            break;
        default:
            break;
    }

    for (int i = (window_size_ / 2); i < (count - (window_size_ / 2)); i++) {
        cur_distance = temp_distance_data[i];
        if (cur_distance > temp_distance_data[i + 1]) {
            diff_1 = cur_distance - temp_distance_data[i + 1];
        } else {
            diff_1 = temp_distance_data[i + 1] - cur_distance;
        }

        if (cur_distance > temp_distance_data[i - 1]) {
            diff_2 = cur_distance - temp_distance_data[i - 1];
        } else {
            diff_2 = temp_distance_data[i - 1] - cur_distance;
        }

        if ((diff_1 < abnormal_point_diff) && (diff_2 < abnormal_point_diff)) {
            if (cur_distance > 500) {
                continue;
            }
        }

        int half_window = window_size_ / 2;
        for (int pos = (i - half_window); pos < i + half_window + 1; pos++) {
            // Avoid comparing the point cloud with itself, skip filtering if
            // the current distance is 0
            if (pos == i || cur_distance == 0) {
                continue;
            }

            point_distance_cmp = temp_distance_data[pos];
            // Calculate the tangent of the angle between the current point and
            // adjacent points
            tan_angle = (cur_distance * angle_res_sin) /
                        (point_distance_cmp - cur_distance * angle_res_cos);
            if ((tan_angle < tan_angle_threshold) &&
                (tan_angle > (-1) * tan_angle_threshold)) {
                if (pos < i)
                    sum_left_abnormal++;  // Accumulate the number of abnormal
                                          // points on the left side
                else
                    sum_right_abnormal++;  // Accumulate the number of abnormal
                                           // points on the right side
            }

            // If the number of abnormal points on either side exceeds the
            // threshold, consider it trailing/noise
            if ((sum_left_abnormal >= point_num_threshold) ||
                (sum_right_abnormal >= point_num_threshold)) {
                for (int num = (-1) * neighbors; num <= neighbors; num++) {
                    ranges[i + num] =
                        0;  // Filter out this point and its neighboring points
                            // as trailing/noise
                }

                break;
            }
        }

        sum_left_abnormal = 0;
        sum_right_abnormal = 0;
    }
}

std::shared_ptr<Frame> OutlierRemovalFilterImpl::process(
    std::shared_ptr<Frame> frame) const {
    if (frame->type() != LidarFrameType::SCAN) {
        // MUST be scan frame
        return frame;
    }

    auto scan_frame = std::dynamic_pointer_cast<ScanFrame>(frame);
    auto scan = scan_frame->toScan();

    auto new_scan_impl = std::make_unique<detail::ScanFrameImpl>();

    filter(scan.ranges);
    new_scan_impl->setRanges(scan.ranges);
    auto new_scan_frame = std::make_shared<ScanFrame>(std::move(new_scan_impl));
    // copy meta from old scan frame, type, timestamp, etc.
    new_scan_frame->copyMetaFrom(frame);
    return new_scan_frame;
}

}  // namespace ob_lidar::detail
