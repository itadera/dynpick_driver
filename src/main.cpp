/*
 * Copyright (c) 2017, Tokyo Opensource Robotics Kyokai Association
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 2025/04/02 Shunki Itadera (AIST
 * This node is ros2 port of the original dynpick_driver node.
 */

#include <fcntl.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <condition_variable>
#include <mutex>

#define RESET_COMMAND_TRY 3 // It only works when sent several times.

#define DATA_LENGTH 27
#define CALIB_DATA_LENGTH 46

std::mutex m_;
std::condition_variable cv_;
int offset_reset_ = RESET_COMMAND_TRY;

int SetComAttr(int fdc)
{
  int n;

  struct termios term;

  // Set baud rate
  n = tcgetattr(fdc, &term);
  if (n < 0)
    goto over;

  bzero(&term, sizeof(term));

  term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
  term.c_iflag = IGNPAR;
  term.c_oflag = 0;
  term.c_lflag = 0;

  term.c_cc[VINTR] = 0;  /* Ctrl-c */
  term.c_cc[VQUIT] = 0;  /* Ctrl-? */
  term.c_cc[VERASE] = 0; /* del */
  term.c_cc[VKILL] = 0;  /* @ */
  term.c_cc[VEOF] = 4;   /* Ctrl-d */
  term.c_cc[VTIME] = 0;
  term.c_cc[VMIN] = 0;
  term.c_cc[VSWTC] = 0;    /* '?0' */
  term.c_cc[VSTART] = 0;   /* Ctrl-q */
  term.c_cc[VSTOP] = 0;    /* Ctrl-s */
  term.c_cc[VSUSP] = 0;    /* Ctrl-z */
  term.c_cc[VEOL] = 0;     /* '?0' */
  term.c_cc[VREPRINT] = 0; /* Ctrl-r */
  term.c_cc[VDISCARD] = 0; /* Ctrl-u */
  term.c_cc[VWERASE] = 0;  /* Ctrl-w */
  term.c_cc[VLNEXT] = 0;   /* Ctrl-v */
  term.c_cc[VEOL2] = 0;    /* '?0' */

  n = tcsetattr(fdc, TCSANOW, &term);
over:
  return (n);
}

bool clearSocket(const int &fdc, char *leftover)
{
  int len = 0;
  int c = 0;
  int length = 255;
  while (len < length)
  {
    c = read(fdc, leftover + len, length - len);
    if (c > 0)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("dynpick_driver"), "More data to clean up; n = %d (%d) ===", c, len);
      len += c;
    }
    else
    {
      RCLCPP_DEBUG(rclcpp::get_logger("dynpick_driver"), "No more data on socket");
      break;
    }
  }
  return true;
}

bool readCharFromSocket(const int &fdc, const int &length, char *reply)
{
  int len = 0;
  int c = 0;
  while (len < length)
  {
    c = read(fdc, reply + len, length - len);
    if (c >= 0)
    {
      len += c;
    }
    else
    {
      RCLCPP_DEBUG(rclcpp::get_logger("dynpick_driver"), "=== need to read more data ... n = %d (%d) ===", c, len);
      continue;
    }
  }
  return true;
}

class DynpickDriver : public rclcpp::Node
{
public:
  DynpickDriver() : Node("dynpick_driver")
  {
    this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("frame_id", "/sensor");
    this->declare_parameter<double>("rate", 1000.0);
    this->declare_parameter<bool>("acquire_calibration", true);
    this->declare_parameter<int>("frequency_div", 1);

    this->get_parameter("device", devname_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("rate", rate_);
    this->get_parameter("acquire_calibration", auto_adjust_);
    this->get_parameter("frequency_div", frq_div_);

    service_ = this->create_service<std_srvs::srv::Trigger>(
        "ft_reset_offset", std::bind(&DynpickDriver::offsetRequest, this, std::placeholders::_1, std::placeholders::_2));
    pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("force", 1000);

    RCLCPP_INFO(this->get_logger(), "Open %s", devname_.c_str());

    fdc_ = open(devname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fdc_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "could not open %s", devname_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sampling time = %f ms", 1.0 / rate_);

    SetComAttr(fdc_);

    char trash[255];
    clearSocket(fdc_, trash);

    if (auto_adjust_)
    {
      write(fdc_, "p", 1);
      char reply[CALIB_DATA_LENGTH];
      readCharFromSocket(fdc_, CALIB_DATA_LENGTH, reply);
      sscanf(reply, "%f,%f,%f,%f,%f,%f", &calib_[0], &calib_[1], &calib_[2], &calib_[3], &calib_[4], &calib_[5]);
      RCLCPP_INFO(this->get_logger(),
                  "Calibration from sensor:\n%.3f LSB/N, %.3f LSB/N, %.3f LSB/N, %.3f LSB/Nm, %.3f LSB/Nm, %.3f LSB/Nm",
                  calib_[0], calib_[1], calib_[2], calib_[3], calib_[4], calib_[5]);
      clearSocket(fdc_, trash);
    }

    if (frq_div_ == 1 || frq_div_ == 2 || frq_div_ == 4 || frq_div_ == 8)
    {
      char cmd[3];
      sprintf(cmd, "%dF", frq_div_);
      write(fdc_, cmd, 2);
      RCLCPP_INFO(this->get_logger(), "Set the frequency divider to %s", cmd);

      write(fdc_, "0F", 2);
      char repl[3];
      readCharFromSocket(fdc_, 3, repl);
      if (repl[0] - '0' != frq_div_)
      {
        RCLCPP_ERROR(this->get_logger(), "Response by sensor is not as expected! Current Filter: %dF", repl[0] - '0');
      }
      clearSocket(fdc_, trash);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "Not setting frequency divider. Parameter out of acceptable values {1,2,4,8}: %d", frq_div_);
    }

    write(fdc_, "R", 1);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / rate_)),
                                     std::bind(&DynpickDriver::publishData, this));
  }

private:
  void publishData()
  {
    char str[256];
    int tick;
    unsigned short data[6];

    geometry_msgs::msg::WrenchStamped msg;

    std::unique_lock<std::mutex> lock(m_);
    if (offset_reset_ <= 0)
    {
      write(fdc_, "R", 1);
      readCharFromSocket(fdc_, DATA_LENGTH, str);

      sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx", &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

      msg.header.frame_id = frame_id_;
      msg.header.stamp = this->now();

      msg.wrench.force.x = (data[0] - 8192) / calib_[0];
      msg.wrench.force.y = (data[1] - 8192) / calib_[1];
      msg.wrench.force.z = (data[2] - 8192) / calib_[2];
      msg.wrench.torque.x = (data[3] - 8192) / calib_[3];
      msg.wrench.torque.y = (data[4] - 8192) / calib_[4];
      msg.wrench.torque.z = (data[5] - 8192) / calib_[5];

      pub_->publish(msg);
      lock.unlock();
    }
    else
    {
      write(fdc_, "O", 1);
      offset_reset_--;
      lock.unlock();
      cv_.notify_all();
    }
  }

  bool offsetRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::unique_lock<std::mutex> lock(m_);
    offset_reset_ = RESET_COMMAND_TRY;
    cv_.wait(lock, []
             { return offset_reset_ <= 0; });
    lock.unlock();
    res->message = "Reset offset command was sent " + std::to_string(RESET_COMMAND_TRY) + " times to the sensor.";
    res->success = true;
    return true;
  }

  int fdc_;
  std::string devname_, frame_id_;
  double rate_;
  bool auto_adjust_;
  int frq_div_;
  float calib_[6] = {1, 1, 1, 1, 1, 1};

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynpickDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
