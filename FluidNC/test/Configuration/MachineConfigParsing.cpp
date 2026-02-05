#include "TestFramework.h"

#include <string>

#include <Configuration/Tokenizer.h>
#include <Configuration/Parser.h>
#include <Machine/Axes.h>
#include <Configuration/ParserHandler.h>
#include <Configuration/Configurable.h>

namespace Configuration {

    // Mirrors Machine::Start from MachineConfig.h for testing machine config YAML shape.
    class StartLike : public Configurable {
    public:
        bool must_home          = true;
        bool deactivate_parking  = false;
        bool check_limits       = true;

        void validate() override {}
        void group(HandlerBase& handler) override {
            handler.item("must_home", must_home);
            handler.item("deactivate_parking", deactivate_parking);
            handler.item("check_limits", check_limits);
        }
    };

    // Mirrors top-level items and one section of MachineConfig for testing machine config parsing.
    class MachineConfigLike : public Configurable {
    public:
        std::string board                    = "None";
        std::string name                     = "None";
        std::string meta                     = "";
        float       arc_tolerance_mm         = 0.002f;
        float       junction_deviation_mm   = 0.01f;
        bool        verbose_errors           = true;
        bool        report_inches            = false;
        int32_t     planner_blocks           = 16;
        bool        enable_parking_override_control = false;
        bool        use_line_numbers         = false;
        StartLike*  start                    = nullptr;

        ~MachineConfigLike() {
            delete start;
        }

        void validate() override {}
        void group(HandlerBase& handler) override {
            handler.item("board", board);
            handler.item("name", name);
            handler.item("meta", meta);
            handler.section("start", start);
            handler.item("arc_tolerance_mm", arc_tolerance_mm, 0.001f, 1.0f);
            handler.item("junction_deviation_mm", junction_deviation_mm, 0.01f, 1.0f);
            handler.item("verbose_errors", verbose_errors);
            handler.item("report_inches", report_inches);
            handler.item("planner_blocks", planner_blocks, 10, 120);
            handler.item("enable_parking_override_control", enable_parking_override_control);
            handler.item("use_line_numbers", use_line_numbers);
        }
    };

    static void ParseMachineTopLevel(const char* yaml, MachineConfigLike& out) {
        Parser        p(yaml);
        ParserHandler handler(p);
        handler.enterSection("machine", &out);
    }

    Test(MachineConfigParsing, TopLevelItems) {
        const char* yaml = "name: MyBoard\n"
                           "board: ESP32-Dev\n"
                           "meta: test config\n"
                           "arc_tolerance_mm: 0.005\n"
                           "junction_deviation_mm: 0.02\n"
                           "verbose_errors: false\n"
                           "report_inches: true\n"
                           "planner_blocks: 32\n"
                           "enable_parking_override_control: true\n"
                           "use_line_numbers: true\n";

        MachineConfigLike cfg;
        ParseMachineTopLevel(yaml, cfg);

        Assert(cfg.name == "MyBoard", "name");
        Assert(cfg.board == "ESP32-Dev", "board");
        Assert(cfg.meta == "test config", "meta");
        Assert(cfg.arc_tolerance_mm > 0.004f && cfg.arc_tolerance_mm < 0.006f, "arc_tolerance_mm");
        Assert(cfg.junction_deviation_mm > 0.019f && cfg.junction_deviation_mm < 0.021f, "junction_deviation_mm");
        Assert(cfg.verbose_errors == false, "verbose_errors");
        Assert(cfg.report_inches == true, "report_inches");
        Assert(cfg.planner_blocks == 32, "planner_blocks");
        Assert(cfg.enable_parking_override_control == true, "enable_parking_override_control");
        Assert(cfg.use_line_numbers == true, "use_line_numbers");
    }

    Test(MachineConfigParsing, StartSection) {
        const char* yaml = "name: Test\n"
                           "board: None\n"
                           "start:\n"
                           "  must_home: false\n"
                           "  deactivate_parking: true\n"
                           "  check_limits: false\n";

        MachineConfigLike cfg;
        ParseMachineTopLevel(yaml, cfg);

        Assert(cfg.start != nullptr, "start section created");
        Assert(cfg.start->must_home == false, "must_home");
        Assert(cfg.start->deactivate_parking == true, "deactivate_parking");
        Assert(cfg.start->check_limits == false, "check_limits");
    }

    Test(MachineConfigParsing, StartSectionWithTopLevel) {
        const char* yaml = "board: MyBoard\n"
                           "start:\n"
                           "  must_home: true\n"
                           "  check_limits: true\n"
                           "name: MyMachine\n"
                           "planner_blocks: 24\n";

        MachineConfigLike cfg;
        ParseMachineTopLevel(yaml, cfg);

        Assert(cfg.board == "MyBoard", "board");
        Assert(cfg.name == "MyMachine", "name");
        Assert(cfg.planner_blocks == 24, "planner_blocks");
        Assert(cfg.start != nullptr, "start section");
        Assert(cfg.start->must_home == true, "must_home");
        Assert(cfg.start->check_limits == true, "check_limits");
    }

}
