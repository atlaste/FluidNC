idf_component_register(SRCS Authentication.cpp Mdns.cpp NotificationsService.cpp OTA.cpp TelnetClient.cpp TelnetServer.cpp WebClient.cpp 
                            WebCommands.cpp WebServer.cpp WifiConfig.cpp WSChannel.cpp
                       INCLUDE_DIRS "." "../.." "../../include"
                       REQUIRES esp_driver_uart)
