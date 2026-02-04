// Copyright (c) 2024 - FluidNC
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Compensated2D.h"
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
::Kinematics::Compensated2D* compensated2D = nullptr;

namespace Kinematics {

    void Compensated2D::group(Configuration::HandlerBase& handler) {
        handler.item("x_min", _x_min, -10000.0f, 10000.0f);
        handler.item("x_max", _x_max, -10000.0f, 10000.0f);
        handler.item("y_min", _y_min, -10000.0f, 10000.0f);
        handler.item("y_max", _y_max, -10000.0f, 10000.0f);
        handler.item("x_count", _x_count, 2, 100);
        handler.item("y_count", _y_count, 2, 100);
        handler.item("file", _filename);

        // Handle the wrapped kinematic using the factory
        KinematicsFactory::factory(handler, _wrapped);
    }

    void Compensated2D::afterParse() {
        // Default to Cartesian if no wrapped kinematic specified
        if (_wrapped == nullptr) {
            _wrapped = new Cartesian("Cartesian");
        }
        _wrapped->afterParse();

        // Allocate the offset grid
        _offsets.resize(_x_count * _y_count, 0.0f);

        // Set global pointer for command access
        compensated2D = this;
    }

    void Compensated2D::validate() {
        Assert(_x_max > _x_min, "Compensated2D x_max must be greater than x_min");
        Assert(_y_max > _y_min, "Compensated2D y_max must be greater than y_min");
        Assert(_x_count >= 2, "Compensated2D x_count must be at least 2");
        Assert(_y_count >= 2, "Compensated2D y_count must be at least 2");
        Assert(_wrapped != nullptr, "Compensated2D requires a wrapped kinematic");
        _wrapped->validate();
    }

    void Compensated2D::init() {
        log_info("Kinematic system: " << name() << " wrapping " << _wrapped->name());
        log_info("  X range: " << _x_min << " to " << _x_max << " mm, " << _x_count << " points");
        log_info("  Y range: " << _y_min << " to " << _y_max << " mm, " << _y_count << " points");
        log_info("  Grid size: " << (_x_count * _y_count) << " points");

        // Load compensation data
        load();

        _wrapped->init();
    }

    void Compensated2D::init_position() {
        _wrapped->init_position();
    }

    float Compensated2D::getOffset(int ix, int iy) const {
        if (ix < 0 || ix >= _x_count || iy < 0 || iy >= _y_count) {
            return 0.0f;
        }
        return _offsets[iy * _x_count + ix];
    }

    float Compensated2D::getOffsetClamped(int ix, int iy) const {
        ix = std::clamp(ix, 0, int(_x_count - 1));
        iy = std::clamp(iy, 0, int(_y_count - 1));
        return _offsets[iy * _x_count + ix];
    }

    // Catmull-Rom cubic interpolation
    float Compensated2D::cubicInterpolate(float t, float p0, float p1, float p2, float p3) const {
        // Catmull-Rom spline formula
        return p1 + 0.5f * t * (p2 - p0 + t * (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3 +
                                               t * (3.0f * (p1 - p2) + p3 - p0)));
    }

    float Compensated2D::bicubicInterpolate(int ix, int iy, float tx, float ty) const {
        // Sample 4x4 grid around the point (with edge clamping)
        float p[4][4];
        for (int j = -1; j <= 2; j++) {
            for (int i = -1; i <= 2; i++) {
                p[j + 1][i + 1] = getOffsetClamped(ix + i, iy + j);
            }
        }

        // Interpolate along X for each row
        float col[4];
        for (int j = 0; j < 4; j++) {
            col[j] = cubicInterpolate(tx, p[j][0], p[j][1], p[j][2], p[j][3]);
        }

        // Interpolate along Y
        return cubicInterpolate(ty, col[0], col[1], col[2], col[3]);
    }

    float Compensated2D::interpolate(float x, float y) const {
        if (_offsets.empty() || _x_count < 2 || _y_count < 2) {
            return 0.0f;
        }

        // Calculate grid step sizes
        float x_step = (_x_max - _x_min) / (_x_count - 1);
        float y_step = (_y_max - _y_min) / (_y_count - 1);

        // Calculate fractional grid position
        float fx = (x - _x_min) / x_step;
        float fy = (y - _y_min) / y_step;

        // Get integer grid indices (clamped to valid interpolation range)
        int ix = std::clamp(int(fx), 0, int(_x_count - 2));
        int iy = std::clamp(int(fy), 0, int(_y_count - 2));

        // Get fractional part for interpolation
        float tx = fx - ix;
        float ty = fy - iy;

        // Clamp fractional part to [0, 1]
        tx = std::clamp(tx, 0.0f, 1.0f);
        ty = std::clamp(ty, 0.0f, 1.0f);

        return bicubicInterpolate(ix, iy, tx, ty);
    }

    bool Compensated2D::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        float compensated[MAX_N_AXIS];
        memcpy(compensated, target, sizeof(float) * MAX_N_AXIS);

        // Apply Z compensation based on X,Y position
        compensated[Z_AXIS] += interpolate(target[X_AXIS], target[Y_AXIS]);

        return _wrapped->cartesian_to_motors(compensated, pl_data, position);
    }

    void Compensated2D::motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) {
        _wrapped->motors_to_cartesian(cartesian, motors, n_axis);
        // Note: We don't reverse-compensate here - reported position is motor position
    }

    bool Compensated2D::transform_cartesian_to_motors(float* motors, float* cartesian) {
        float compensated[MAX_N_AXIS];
        memcpy(compensated, cartesian, sizeof(float) * MAX_N_AXIS);

        // Apply Z compensation based on X,Y position
        compensated[Z_AXIS] += interpolate(cartesian[X_AXIS], cartesian[Y_AXIS]);

        return _wrapped->transform_cartesian_to_motors(motors, compensated);
    }

    // Delegate all other methods to wrapped kinematic
    void Compensated2D::constrain_jog(float* cartesian, plan_line_data_t* pl_data, float* position) {
        _wrapped->constrain_jog(cartesian, pl_data, position);
    }

    bool Compensated2D::invalid_line(float* cartesian) {
        return _wrapped->invalid_line(cartesian);
    }

    bool Compensated2D::invalid_arc(float*            target,
                                    plan_line_data_t* pl_data,
                                    float*            position,
                                    float             center[3],
                                    float             radius,
                                    axis_t            caxes[3],
                                    bool              is_clockwise_arc,
                                    uint32_t          rotations) {
        return _wrapped->invalid_arc(target, pl_data, position, center, radius, caxes, is_clockwise_arc, rotations);
    }

    bool Compensated2D::canHome(AxisMask axisMask) {
        return _wrapped->canHome(axisMask);
    }

    void Compensated2D::releaseMotors(AxisMask axisMask, MotorMask motors) {
        _wrapped->releaseMotors(axisMask, motors);
    }

    bool Compensated2D::limitReached(AxisMask& axisMask, MotorMask& motors, MotorMask limited) {
        return _wrapped->limitReached(axisMask, motors, limited);
    }

    bool Compensated2D::kinematics_homing(AxisMask& axisMask) {
        return _wrapped->kinematics_homing(axisMask);
    }

    void Compensated2D::homing_move(AxisMask axes, MotorMask motors, Machine::Homing::Phase phase, uint32_t settling_ms) {
        _wrapped->homing_move(axes, motors, phase, settling_ms);
    }

    void Compensated2D::set_homed_mpos(float* mpos) {
        _wrapped->set_homed_mpos(mpos);
    }

    // File I/O
    bool Compensated2D::load() {
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

            log_info("Loading 2D compensation from " << _filename);

            // Parse grid from YAML
            // Format: row0:\n  - <val>\n  - <val>\n...\nrow1:\n...
            Configuration::Parser        parser(std::string_view{ buffer.get(), filesize });
            Configuration::ParserHandler handler(parser);

            parser.Tokenize();
            while (parser._token._state != Configuration::TokenState::Eof) {
                std::string key(parser.key());

                // Parse row sections like "row0", "row1", etc.
                if (key.length() > 3 && key.substr(0, 3) == "row") {
                    int row_idx = std::stoi(key.substr(3));
                    if (row_idx >= 0 && row_idx < _y_count) {
                        // Parse the row values
                        parser.Tokenize();  // Enter section
                        int col_idx = 0;
                        while (parser._token._state == Configuration::TokenState::Matching ||
                               parser._token._state == Configuration::TokenState::Held) {
                            std::string subkey(parser.key());
                            // Keys should be "c0", "c1", etc.
                            if (subkey.length() > 1 && subkey[0] == 'c') {
                                col_idx = std::stoi(subkey.substr(1));
                                if (col_idx >= 0 && col_idx < _x_count) {
                                    _offsets[row_idx * _x_count + col_idx] = parser.floatValue();
                                }
                            }
                            if (parser._token._state == Configuration::TokenState::Held) {
                                parser._token._state = Configuration::TokenState::Matching;
                            }
                            parser.Tokenize();
                            if (parser._token._indent == 0) {
                                break;
                            }
                        }
                        continue;
                    }
                }

                if (parser._token._state == Configuration::TokenState::Held) {
                    parser._token._state = Configuration::TokenState::Matching;
                }
                if (parser._token._state == Configuration::TokenState::Matching) {
                    parser.Tokenize();
                }
            }

            _loaded = true;
            log_info("Loaded " << _x_count << "x" << _y_count << " compensation grid");
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

    bool Compensated2D::save() {
        try {
            FileStream file(_filename, "wb", "");

            // Write grid as YAML
            // Format: row0:\n  c0: <val>\n  c1: <val>\n...\nrow1:\n...
            for (int y = 0; y < _y_count; y++) {
                char buf[64];
                int len = snprintf(buf, sizeof(buf), "row%d:\n", y);
                file.write((uint8_t*)buf, len);

                for (int x = 0; x < _x_count; x++) {
                    float val = _offsets[y * _x_count + x];
                    len = snprintf(buf, sizeof(buf), "  c%d: %.6f\n", x, val);
                    file.write((uint8_t*)buf, len);
                }
            }

            file.flush();
            log_info("Saved " << _x_count << "x" << _y_count << " compensation grid to " << _filename);
            return true;

        } catch (const std::exception& e) {
            log_error("Failed to save compensation: " << e.what());
            return false;
        } catch (...) {
            log_error("Failed to save compensation");
            return false;
        }
    }

    void Compensated2D::setOffset(int ix, int iy, float z_offset) {
        if (ix >= 0 && ix < _x_count && iy >= 0 && iy < _y_count) {
            _offsets[iy * _x_count + ix] = z_offset;
            save();  // Auto-save on change
        }
    }

    void Compensated2D::clearOffsets() {
        std::fill(_offsets.begin(), _offsets.end(), 0.0f);
        save();  // Auto-save on change
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<Compensated2D> registration("Compensated2D");
    }

}  // namespace Kinematics
