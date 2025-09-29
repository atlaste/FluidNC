idf_component_register(SRCS AfterParse.cpp Completer.cpp GCodeParam.cpp Generator.cpp JsonGenerator.cpp LegacySettingRegistry.cpp Parser.cpp 
                            RuntimeSetting.cpp Tokenizer.cpp Validator.cpp
                       INCLUDE_DIRS "." "../.." "../../include"
                       REQUIRES driver fatfs vfs)
