// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Compensated1D.h"
#include "Cartesian.h"
#include "Machine/MachineConfig.h"
#include "Machine/Axes.h"
#include "FileStream.h"
#include "Logging.h"
#include "Configuration/Parser.h"
#include "Configuration/ParserHandler.h"
#include "Configuration/Generator.h"

#include <cstring>
#include <memory>
#include <algorithm>
#include <cmath>

// Global instance pointer for command access
::Kinematics::Compensated1D* compensated1D = nullptr;

namespace Kinematics {

    void Compensated1D::group(Configuration::HandlerBase& handler) {
        handler.item("z_min", _z_min, -10000.0f, 0.0f);
        handler.item("z_max", _z_max, 0.0f, 10000.0f);
        handler.item("granularity", _granularity, 0.001f, 10.0f);
        handler.item("file", _filename);

        // Handle the wrapped kinematic using the factory
        KinematicsFactory::factory(handler, _wrapped);
    }

    void Compensated1D::afterParse() {
        // Default to Cartesian if no wrapped kinematic specified
        if (_wrapped == nullptr) {
            _wrapped = new Cartesian("Cartesian");
        }
        _wrapped->afterParse();

        // Allocate the offset table
        int count = int((_z_max - _z_min) / _granularity) + 1;
        _offsets.resize(count, 0.0f);

        // Set global pointer for command access
        compensated1D = this;
    }

    void Compensated1D::validate() {
        Assert(_z_max > _z_min, "Compensated1D z_max must be greater than z_min");
        Assert(_granularity > 0.0f, "Compensated1D granularity must be positive");
        Assert(_wrapped != nullptr, "Compensated1D requires a wrapped kinematic");
        _wrapped->validate();
    }

    void Compensated1D::init() {
        log_info("Kinematic system: " << name() << " wrapping " << _wrapped->name());
        log_info("  Z range: " << _z_min << " to " << _z_max << " mm, granularity: " << _granularity << " mm");
        log_info("  Table size: " << _offsets.size() << " points");

        // Load compensation data
        load();

        _wrapped->init();
    }

    void Compensated1D::init_position() {
        _wrapped->init_position();
    }

    int Compensated1D::zToIndex(float z) const {
        return int((z - _z_min) / _granularity);
    }

    float Compensated1D::indexToZ(int index) const {
        return _z_min + index * _granularity;
    }

    float Compensated1D::interpolate(float z) const {
        if (_offsets.empty()) {
            return 0.0f;
        }

        // Calculate fractional index
        float idx_f  = (z - _z_min) / _granularity;
        int   idx    = int(idx_f);
        int   maxIdx = int(_offsets.size()) - 1;

        // Clamp to valid range
        if (idx < 0) {
            return _offsets[0];
        }
        if (idx >= maxIdx) {
            return _offsets[maxIdx];
        }

        // Linear interpolation between adjacent points
        float t = idx_f - idx;
        return _offsets[idx] * (1.0f - t) + _offsets[idx + 1] * t;
    }

    bool Compensated1D::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        float compensated[MAX_N_AXIS];
        memcpy(compensated, target, sizeof(float) * MAX_N_AXIS);

        // Apply X compensation based on Z position
        compensated[X_AXIS] += interpolate(target[Z_AXIS]);

        return _wrapped->cartesian_to_motors(compensated, pl_data, position);
    }

    void Compensated1D::motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) {
        _wrapped->motors_to_cartesian(cartesian, motors, n_axis);
        // Note: We don't reverse-compensate here - reported position is motor position
    }

    bool Compensated1D::transform_cartesian_to_motors(float* motors, float* cartesian) {
        float compensated[MAX_N_AXIS];
        memcpy(compensated, cartesian, sizeof(float) * MAX_N_AXIS);

        // Apply X compensation based on Z position
        compensated[X_AXIS] += interpolate(cartesian[Z_AXIS]);

        return _wrapped->transform_cartesian_to_motors(motors, compensated);
    }

    // Delegate all other methods to wrapped kinematic
    void Compensated1D::constrain_jog(float* cartesian, plan_line_data_t* pl_data, float* position) {
        _wrapped->constrain_jog(cartesian, pl_data, position);
    }

    bool Compensated1D::invalid_line(float* cartesian) {
        return _wrapped->invalid_line(cartesian);
    }

    bool Compensated1D::invalid_arc(float*            target,
                                    plan_line_data_t* pl_data,
                                    float*            position,
                                    float             center[3],
                                    float             radius,
                                    axis_t            caxes[3],
                                    bool              is_clockwise_arc,
                                    uint32_t          rotations) {
        return _wrapped->invalid_arc(target, pl_data, position, center, radius, caxes, is_clockwise_arc, rotations);
    }

    bool Compensated1D::canHome(AxisMask axisMask) {
        return _wrapped->canHome(axisMask);
    }

    void Compensated1D::releaseMotors(AxisMask axisMask, MotorMask motors) {
        _wrapped->releaseMotors(axisMask, motors);
    }

    bool Compensated1D::limitReached(AxisMask& axisMask, MotorMask& motors, MotorMask limited) {
        return _wrapped->limitReached(axisMask, motors, limited);
    }

    bool Compensated1D::kinematics_homing(AxisMask& axisMask) {
        return _wrapped->kinematics_homing(axisMask);
    }

    void Compensated1D::homing_move(AxisMask axes, MotorMask motors, Machine::Homing::Phase phase, uint32_t settling_ms) {
        _wrapped->homing_move(axes, motors, phase, settling_ms);
    }

    void Compensated1D::set_homed_mpos(float* mpos) {
        _wrapped->set_homed_mpos(mpos);
    }

    // File I/O - Load sparse points and interpolate into fixed array
    bool Compensated1D::load() {
        _loaded = false;

        // Reset all offsets to zero
        std::fill(_offsets.begin(), _offsets.end(), 0.0f);

        try {
            FileStream file(_filename, "rb", "");

            auto filesize = file.size();
            if (filesize <= 0) {
                log_info("Compensation file " << _filename << " is empty or doesn't exist, using zero offsets");
                _loaded = true;
                return true;
            }

            auto buffer      = std::make_unique<char[]>(filesize + 1);
            buffer[filesize] = '\0';
            auto actual      = file.read(buffer.get(), filesize);
            if (actual != filesize) {
                log_error("Compensation file read error");
                return false;
            }

            log_info("Loading 1D compensation from " << _filename);

            // Parse sparse points from YAML
            // Format: offsets:\n  - z: <value>\n    x: <value>\n  ...
            std::vector<std::pair<float, float>> sparse_points;

            Configuration::Parser        parser(std::string_view{ buffer.get(), filesize });
            Configuration::ParserHandler handler(parser);

            parser.Tokenize();
            while (parser._token._state != Configuration::TokenState::Eof) {
                std::string key(parser.key());

                if (key.length() > 0 && key[0] == 'p') {
                    // Parse point sections like "p0", "p1", etc.
                    float z_val = 0.0f, x_val = 0.0f;

                    // Need to manually parse this section
                    parser.Tokenize();  // Enter section
                    while (parser._token._state == Configuration::TokenState::Matching ||
                           parser._token._state == Configuration::TokenState::Held) {
                        std::string subkey(parser.key());
                        if (subkey == "z") {
                            z_val = parser.floatValue();
                        } else if (subkey == "x") {
                            x_val = parser.floatValue();
                        }
                        if (parser._token._state == Configuration::TokenState::Held) {
                            parser._token._state = Configuration::TokenState::Matching;
                        }
                        parser.Tokenize();
                        // Check if we've exited the section (indentation decreased)
                        if (parser._token._indent == 0) {
                            break;
                        }
                    }
                    sparse_points.push_back({ z_val, x_val });
                    continue;
                }

                if (parser._token._state == Configuration::TokenState::Held) {
                    parser._token._state = Configuration::TokenState::Matching;
                }
                if (parser._token._state == Configuration::TokenState::Matching) {
                    parser.Tokenize();
                }
            }

            // Sort by Z position
            std::sort(sparse_points.begin(), sparse_points.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });

            // Interpolate sparse points into fixed array
            if (!sparse_points.empty()) {
                for (size_t i = 0; i < _offsets.size(); i++) {
                    float z = indexToZ(i);

                    // Find surrounding points
                    auto it = std::lower_bound(sparse_points.begin(), sparse_points.end(), z,
                                               [](const auto& p, float v) { return p.first < v; });

                    if (it == sparse_points.begin()) {
                        _offsets[i] = sparse_points.front().second;
                    } else if (it == sparse_points.end()) {
                        _offsets[i] = sparse_points.back().second;
                    } else {
                        auto prev = std::prev(it);
                        float t   = (z - prev->first) / (it->first - prev->first);
                        _offsets[i] = prev->second * (1.0f - t) + it->second * t;
                    }
                }
            }

            _loaded = true;
            log_info("Loaded " << sparse_points.size() << " compensation points");
            return true;

        } catch (const std::exception& e) {
            log_info("Compensation file " << _filename << " not found: " << e.what() << ", using zero offsets");
            _loaded = true;
            return true;
        } catch (...) {
            log_info("Compensation file " << _filename << " not found, using zero offsets");
            _loaded = true;
            return true;
        }
    }

    bool Compensated1D::save() {
        try {
            FileStream file(_filename, "wb", "");

            // Find non-zero points to save (sparse format)
            // We'll save points at regular intervals where offset is non-zero
            // or at inflection points

            std::vector<std::pair<float, float>> points;

            // Sample at reasonable intervals for sparse storage
            const float sample_interval = std::max(_granularity * 10.0f, 1.0f);  // At least every 1mm or 10x granularity
            float       last_z          = _z_min - sample_interval * 2;
            float       last_offset     = 0.0f;
            bool        last_was_zero   = true;

            for (size_t i = 0; i < _offsets.size(); i++) {
                float z      = indexToZ(i);
                float offset = _offsets[i];

                bool is_zero     = (std::abs(offset) < 0.0001f);
                bool is_endpoint = (i == 0 || i == _offsets.size() - 1);

                // Save point if:
                // - It's an endpoint
                // - Transitioning from zero to non-zero or vice versa
                // - Enough distance since last point and offset changed
                if (is_endpoint || (is_zero != last_was_zero) ||
                    (!is_zero && z - last_z >= sample_interval && std::abs(offset - last_offset) > 0.0001f)) {
                    points.push_back({ z, offset });
                    last_z        = z;
                    last_offset   = offset;
                    last_was_zero = is_zero;
                }
            }

            // Write sparse points as YAML
            // Format: p0:\n  z: <value>\n  x: <value>\n...
            int idx = 0;
            for (const auto& pt : points) {
                char buf[128];
                int len = snprintf(buf, sizeof(buf), "p%d:\n  z: %.4f\n  x: %.6f\n", idx++, pt.first, pt.second);
                file.write((uint8_t*)buf, len);
            }

            file.flush();
            log_info("Saved " << points.size() << " compensation points to " << _filename);
            return true;

        } catch (const std::exception& e) {
            log_error("Failed to save compensation: " << e.what());
            return false;
        } catch (...) {
            log_error("Failed to save compensation");
            return false;
        }
    }

    void Compensated1D::setOffset(float z, float x_offset) {
        int idx = zToIndex(z);
        if (idx >= 0 && idx < int(_offsets.size())) {
            _offsets[idx] = x_offset;
            save();  // Auto-save on change
        }
    }

    void Compensated1D::clearOffsets() {
        std::fill(_offsets.begin(), _offsets.end(), 0.0f);
        save();  // Auto-save on change
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<Compensated1D> registration("Compensated1D");
    }

}  // namespace Kinematics
