1.
    在CmakeLists.txt中安装config:
    install(DIRECTORY 
        launch
        config
        DESTINATION shared/${PROJECT_NAME})