#include <chrono>
#include <iostream>
#include <random>
#include "FortuneAlgorithm.h"

std::vector<Vector2D> generatePoints(int count) {
    uint64_t seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    std::vector<Vector2D> points;
    for (int i = 0; i < count; ++i)
        points.emplace_back(distribution(generator), distribution(generator));

    return points;
}

VoronoiDiagram generateRandomDiagram(std::size_t count) {
    std::vector<Vector2D> points = generatePoints(count);
    FortuneAlgorithm algorithm(points);
    algorithm.construct();
    algorithm.bound(Frame{-0.05, -0.05, 1.05, 1.05});
    return algorithm.getDiagram();
}

int main() {
    for (int count = 100; count < 100000; count+=100)
    {
        auto start = clock();
        auto diagram = generateRandomDiagram(count);
        auto time = clock() - start;
        std::cout << count << ' ' << time << '\n';
    }
    return 0;
}
