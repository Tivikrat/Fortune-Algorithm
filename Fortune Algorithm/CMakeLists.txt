cmake_minimum_required(VERSION 3.15)
project(Fortune_Algorithm)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(Fortune_Algorithm
        ArcNode.h
        BeachLine.h
        Frame.h
        Event.h
        FortuneAlgorithm.h
        main.cpp
        PriorityQueue.h
        Vector2D.h
        VoronoiDiagram.h)
