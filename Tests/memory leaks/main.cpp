#include <chrono>
#include <iostream>
#include <random>
#include "FortuneAlgorithm.h"
#include "windows.h"
#include "psapi.h"

long long getMemoryUsage() {
    PROCESS_MEMORY_COUNTERS pmc;
    GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
    return pmc.WorkingSetSize;
}

std::vector<Vector2D> generatePoints(int count) {
    uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    std::vector<Vector2D> points;
    for (int i = 0; i < count; ++i)
        points.emplace_back(distribution(generator), distribution(generator));

    return points;
}

void generateRandomDiagram(std::size_t count) {
    std::vector<Vector2D> points = generatePoints(count);
    auto start = clock();
    FortuneAlgorithm algorithm(points);
    algorithm.construct();
    algorithm.bound(Frame{-0.05, -0.05, 1.05, 1.05});
    auto time = clock() - start;
    std::cout << count << " points for " << time << " ms using " << getMemoryUsage() << " bytes" << std::endl;
    algorithm.getDiagram();
}

int main() {
    while (true) {
        generateRandomDiagram(64000);
    }
    system("pause");
    return 0;
}
