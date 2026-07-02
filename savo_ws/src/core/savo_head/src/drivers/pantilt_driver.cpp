#include "savo_head/drivers/pantilt_driver.hpp"

#include <sstream>
#include <stdexcept>

namespace savo_head
{

PanTiltDriver::PanTiltDriver(PanTiltDriverConfig config)
: PanTiltDriver(
    config.normalized(),
    std::make_unique<Pca9685Driver>(config.normalized().pca9685_config()))
{
}

PanTiltDriver::PanTiltDriver(
  PanTiltDriverConfig config,
  std::unique_ptr<Pca9685Driver> pca_driver)
: config_(config.normalized()),
  calibration_(config_.calibration.normalized()),
  pca_driver_(std::move(pca_driver)),
  state_{
    calibration_.pan.center_deg,
    calibration_.tilt.center_deg,
    HeadMode::kIdle,
    driver_status_for_backend(config_.dryrun(), false),
    0.0,
    "init"}
{
  if (!pca_driver_) {
    pca_driver_ = std::make_unique<Pca9685Driver>(config_.pca9685_config());
  }
}

PanTiltDriver::~PanTiltDriver()
{
  try {
    close();
  } catch (...) {
  }
}

PanTiltDriver::PanTiltDriver(PanTiltDriver && other) noexcept
{
  *this = std::move(other);
}

PanTiltDriver & PanTiltDriver::operator=(PanTiltDriver && other) noexcept
{
  if (this == &other) {
    return *this;
  }

  try {
    close();
  } catch (...) {
  }

  config_ = other.config_;
  calibration_ = other.calibration_;
  pca_driver_ = std::move(other.pca_driver_);
  opened_ = other.opened_;
  state_ = other.state_;
  last_outputs_ = std::move(other.last_outputs_);
  last_error_ = std::move(other.last_error_);

  other.opened_ = false;
  other.last_error_.reset();

  return *this;
}

void PanTiltDriver::open()
{
  if (opened_) {
    return;
  }

  clear_error();

  try {
    pca_driver_->open();
    opened_ = true;

    update_state(
      state_.pan_deg,
      state_.tilt_deg,
      state_.mode,
      driver_status_for_backend(config_.dryrun(), opened_),
      CommandSource::kSystem,
      state_.stamp_s);

    if (config_.center_on_open) {
      const auto centered = center(CommandSource::kSystem, 0.0);
      (void)centered;
    }
  } catch (const std::exception & exc) {
    set_error(exc.what());
    opened_ = false;
    throw;
  }
}

void PanTiltDriver::close()
{
  if (opened_ && config_.center_on_close) {
    try {
      const auto centered = center(CommandSource::kSystem, state_.stamp_s);
      (void)centered;
    } catch (const std::exception & exc) {
      set_error(exc.what());
    }
  }

  if (pca_driver_) {
    pca_driver_->close();
  }

  opened_ = false;
  update_state(
    state_.pan_deg,
    state_.tilt_deg,
    HeadMode::kIdle,
    driver_status_for_backend(config_.dryrun(), opened_),
    CommandSource::kSystem,
    state_.stamp_s);
}

void PanTiltDriver::ensure_open()
{
  if (!opened_) {
    open();
  }
}

bool PanTiltDriver::opened() const
{
  return opened_;
}

bool PanTiltDriver::dryrun() const
{
  return config_.dryrun();
}

const PanTiltDriverConfig & PanTiltDriver::config() const
{
  return config_;
}

const HeadServoCalibration & PanTiltDriver::calibration() const
{
  return calibration_;
}

PanTiltLimits PanTiltDriver::limits() const
{
  return calibration_.limits();
}

PanTiltState PanTiltDriver::state() const
{
  return state_;
}

const std::vector<ServoOutput> & PanTiltDriver::last_outputs() const
{
  return last_outputs_;
}

std::optional<std::string> PanTiltDriver::last_error() const
{
  return last_error_;
}

Pca9685Driver & PanTiltDriver::pca9685()
{
  return *pca_driver_;
}

const Pca9685Driver & PanTiltDriver::pca9685() const
{
  return *pca_driver_;
}

void PanTiltDriver::clear_error()
{
  last_error_.reset();

  if (pca_driver_) {
    pca_driver_->clear_error();
  }
}

ServoOutput PanTiltDriver::set_axis(
  const std::string & axis,
  int angle_deg,
  HeadMode mode,
  CommandSource source,
  double stamp_s)
{
  ensure_open();

  try {
    const auto output = write_axis(axis, angle_deg);
    last_outputs_ = {output};

    if (axis == "pan") {
      update_state(
        output.angle_deg,
        state_.tilt_deg,
        mode,
        driver_status_for_backend(config_.dryrun(), opened_),
        source,
        stamp_s);
    } else if (axis == "tilt") {
      update_state(
        state_.pan_deg,
        output.angle_deg,
        mode,
        driver_status_for_backend(config_.dryrun(), opened_),
        source,
        stamp_s);
    } else {
      throw std::invalid_argument("unknown head axis: " + axis);
    }

    clear_error();
    return output;
  } catch (const std::exception & exc) {
    set_error(exc.what());
    throw;
  }
}

PanTiltState PanTiltDriver::set_pan_tilt(
  int pan_deg,
  int tilt_deg,
  HeadMode mode,
  CommandSource source,
  double stamp_s)
{
  ensure_open();

  try {
    const auto lim = limits();
    const auto pan = lim.clamp_pan(pan_deg);
    const auto tilt = lim.clamp_tilt(tilt_deg);

    const auto pan_output = write_axis("pan", pan);
    const auto tilt_output = write_axis("tilt", tilt);

    last_outputs_ = {pan_output, tilt_output};

    update_state(
      pan_output.angle_deg,
      tilt_output.angle_deg,
      mode,
      driver_status_for_backend(config_.dryrun(), opened_),
      source,
      stamp_s);

    clear_error();
    return state_;
  } catch (const std::exception & exc) {
    set_error(exc.what());
    throw;
  }
}

PanTiltState PanTiltDriver::center(CommandSource source, double stamp_s)
{
  return set_pan_tilt(
    calibration_.pan.center_deg,
    calibration_.tilt.center_deg,
    HeadMode::kCentering,
    source,
    stamp_s);
}

PanTiltState PanTiltDriver::hold(CommandSource source, double stamp_s)
{
  update_state(
    state_.pan_deg,
    state_.tilt_deg,
    state_.mode,
    driver_status_for_backend(config_.dryrun(), opened_),
    source,
    stamp_s);

  return state_;
}

PanTiltState PanTiltDriver::stop(CommandSource source, double stamp_s)
{
  update_state(
    state_.pan_deg,
    state_.tilt_deg,
    HeadMode::kIdle,
    driver_status_for_backend(config_.dryrun(), opened_),
    source,
    stamp_s);

  return state_;
}

PanTiltState PanTiltDriver::apply_command(const PanTiltCommand & command)
{
  if (command.type == CommandType::kStop) {
    return stop(command.source, command.stamp_s);
  }

  if (command.type == CommandType::kHold) {
    return hold(command.source, command.stamp_s);
  }

  const auto target = target_from_command(command, state_, limits());

  return set_pan_tilt(
    target.pan_deg,
    target.tilt_deg,
    target.mode,
    command.source,
    command.stamp_s);
}

std::string PanTiltDriver::status_text() const
{
  std::ostringstream stream;
  stream << "backend=" << config_.backend
         << ";opened=" << (opened_ ? "true" : "false")
         << ";dryrun=" << (dryrun() ? "true" : "false")
         << ";pan=" << state_.pan_deg
         << ";tilt=" << state_.tilt_deg
         << ";mode=" << to_string(state_.mode)
         << ";status=" << to_string(state_.status)
         << ";source=" << state_.source;

  if (last_error_.has_value() && !last_error_->empty()) {
    stream << ";error=" << *last_error_;
  }

  return stream.str();
}

ServoOutput PanTiltDriver::write_axis(const std::string & axis, int angle_deg)
{
  const auto output = angle_to_servo_output(axis, angle_deg, calibration_);
  const auto ticks = pca_driver_->set_servo_pulse_us(output.pca9685_channel, output.pulse_us);

  ServoOutput actual = output;
  actual.ticks = ticks;

  return actual;
}

void PanTiltDriver::set_error(std::string message)
{
  last_error_ = std::move(message);
  state_.status = HeadStatus::kError;
}

void PanTiltDriver::update_state(
  int pan_deg,
  int tilt_deg,
  HeadMode mode,
  HeadStatus status,
  CommandSource source,
  double stamp_s)
{
  state_ = PanTiltState{
    limits().clamp_pan(pan_deg),
    limits().clamp_tilt(tilt_deg),
    mode,
    status,
    stamp_s,
    to_string(source)
  }.normalized(limits());
}

}  // namespace savo_head
