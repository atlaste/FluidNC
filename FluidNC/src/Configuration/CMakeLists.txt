idf_component_register(SRC_DIRS "."
                       SRCS AfterParse.cpp Completer.cpp GCodeParam.cpp Generator.cpp JsonGenerator.cpp LegacySettingRegistry.cpp Parser.cpp 
                            RuntimeSetting.cpp Tokenizer.cpp Validator.cpp
                       INCLUDE_DIRS "."
                       REQUIRES driver fatfs vfs)
